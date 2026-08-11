#!/usr/bin/env python3
"""
measure_walk.py : drive the robot and score the result, without a human.

WHY THIS EXISTS
Every walking test so far has been run by a person watching a Gazebo window
and reporting an impression: "it wiggles", "it flies away", "it moves
randomly". Those descriptions are ambiguous -- drift, slip, ejection and a
correct slow walk look similar through a window, and each has a different
cause and a different fix. Several days were spent guessing between them.

This node replaces the person. It drives the robot itself, watches /odom,
and prints numbers plus a verdict. It does not need a GUI, so it can run
headless and fast, and it can be run repeatedly with different parameters to
tune something without a human in the loop.

PHASES
  1. wait      block until /odom arrives, so we never measure nothing
  2. baseline  no command at all. A robot that moves here is drifting, and
               drift must be measured BEFORE walking or it is impossible to
               tell from travel.
  3. drive     publish a constant forward velocity
  4. settle    stop commanding, let it come to rest

USAGE
  ros2 run ... no. Run directly, with the workspace sourced:

    python3 tools/measure_walk.py --speed 0.03 --drive 20

  Normally invoked by tools/autotest.sh rather than by hand.
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Standard conversion. Kept local so this file has no extra dependency."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    # Clamp: numerical noise can push this just past 1.0 and asin() then
    # raises, killing the measurement run at the last moment.
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class WalkMeasurer(Node):

    def __init__(self, speed: float, baseline_s: float,
                 drive_s: float, settle_s: float) -> None:
        # use_sim_time MUST be set at construction, not afterwards.
        #
        # It was previously applied with set_parameters() just before run().
        # For a moment after that the node's clock still returns WALL time --
        # about 1.7e9 -- and then flips to simulation time, about 50. The
        # first phase computed `start = now()` on one side of that flip and
        # compared against the other, so the baseline loop exited instantly:
        # every run reported a 0.06 s baseline instead of 8 s, and dividing a
        # few millimetres of settling by 0.06 s produced a fake 200 mm/s
        # "drift" that we nearly went chasing.
        super().__init__(
            "walk_measurer",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", value=True)
            ],
        )

        self.speed = speed
        self.baseline_s = baseline_s
        self.drive_s = drive_s
        self.settle_s = settle_s

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Odometry from gazebo_ros_p3d is best-effort in some configurations.
        # Subscribing RELIABLE to a BEST_EFFORT publisher silently matches
        # nothing, which would look exactly like "the robot never moved".
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odom, qos
        )

        # WATCH THE LEGS TOO.
        #
        # Odometry alone cannot tell "legs cycling but feet slipping" apart
        # from "legs barely moving". Both give a stationary body, and they
        # need opposite fixes -- more foot friction versus finding out why
        # the gait is not commanding motion. One number separates them:
        # how far a femur actually travels during the drive phase.
        from sensor_msgs.msg import JointState  # local: keeps imports honest
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joints, 10
        )
        self.joint_lo: dict[str, float] = {}
        self.joint_hi: dict[str, float] = {}
        self.joint_sum: dict[str, float] = {}
        self.joint_n: dict[str, int] = {}

        # THE CONTROLLER'S OWN VIEW.
        #
        # Comparing measured angles against a constant copied out of the
        # xacro tests my assumption about what the gait is asking for, not
        # what it actually asked. JointTrajectoryController publishes both
        # the reference it received and the feedback it measured, so this
        # removes me from the loop entirely: if reference says -1.18 and
        # feedback says -1.61, the controller is failing; if reference
        # ITSELF says -1.61, the gait is commanding the wrong pose and the
        # controller is doing its job perfectly.
        try:
            from control_msgs.msg import JointTrajectoryControllerState
            for ctrl in ("leg_trajectory_controller", "leg_position_controller"):
                self.create_subscription(
                    JointTrajectoryControllerState,
                    f"/{ctrl}/controller_state",
                    self.on_ctrl_state, 10,
                )
        except Exception as exc:  # noqa: BLE001
            print(f"(controller_state unavailable: {exc})")
        self.ref_sum: dict[str, float] = {}
        self.fb_sum: dict[str, float] = {}
        self.ref_n: dict[str, int] = {}

        self.latest: Odometry | None = None
        self.samples: list[tuple[float, float, float, float, float, float]] = []
        self.recording = False

    def on_joints(self, msg) -> None:
        if not self.recording:
            return
        for name, pos in zip(msg.name, msg.position):
            if name not in self.joint_lo or pos < self.joint_lo[name]:
                self.joint_lo[name] = pos
            if name not in self.joint_hi or pos > self.joint_hi[name]:
                self.joint_hi[name] = pos
            self.joint_sum[name] = self.joint_sum.get(name, 0.0) + pos
            self.joint_n[name] = self.joint_n.get(name, 0) + 1

    def on_ctrl_state(self, msg) -> None:
        if not self.recording:
            return
        ref = list(getattr(msg.reference, "positions", []) or [])
        fb = list(getattr(msg.feedback, "positions", []) or [])
        for i, name in enumerate(msg.joint_names):
            if i < len(ref):
                self.ref_sum[name] = self.ref_sum.get(name, 0.0) + ref[i]
                self.ref_n[name] = self.ref_n.get(name, 0) + 1
            if i < len(fb):
                self.fb_sum[name] = self.fb_sum.get(name, 0.0) + fb[i]

    def ctrl_mean_by_segment(self) -> dict:
        out = {}
        for seg in ("coxa", "femur", "tibia"):
            refs, fbs = [], []
            for n, cnt in self.ref_n.items():
                if seg not in n or not cnt:
                    continue
                refs.append(self.ref_sum[n] / cnt)
                if n in self.fb_sum:
                    fbs.append(self.fb_sum[n] / cnt)
            if refs:
                out[seg] = (sum(refs) / len(refs),
                            sum(fbs) / len(fbs) if fbs else float("nan"))
        return out

    # Design stance, from common_properties.xacro.
    # UPDATED for the knee-down IK branch. These were left at the old
    # 1.6190 / -1.1834 after the branch change and reported a healthy robot
    # as being 77 and 135 degrees out of position. A hardcoded copy of a
    # number that lives in another file will go stale; that is now three
    # times in this project. If the robot is sagging,
    # the measured angles differ from these and the DIFFERENCE says which
    # joint is failing to hold -- which body height alone cannot tell you.
    DESIGN = {"coxa": 0.0, "femur": 0.2694, "tibia": 1.1837}

    def mean_by_segment(self) -> dict:
        out = {}
        for seg in ("coxa", "femur", "tibia"):
            vals = [
                self.joint_sum[n] / self.joint_n[n]
                for n in self.joint_sum
                if seg in n and self.joint_n.get(n)
            ]
            if vals:
                out[seg] = sum(vals) / len(vals)
        return out

    def span_deg(self, segment: str) -> float:
        """Largest peak-to-peak excursion of any joint of this segment.

        WHICH JOINT YOU LOOK AT DECIDES WHAT YOU LEARN, and getting this
        wrong cost a round trip:

          coxa  : axis is VERTICAL (0 0 1). Rotating it swings the foot
                  forwards and backwards. This is STRIDE -- the reach that
                  actually propels the body.
          femur : axis is HORIZONTAL (0 1 0). This is LIFT.

        A gait commanded at zero velocity still cycles the legs up and down,
        so femur motion alone proves only that the gait clock is running. If
        femur swings and coxa does not, the robot is stepping on the spot
        because it was never told to go anywhere -- which is a completely
        different fault from feet that slip.
        """
        best = 0.0
        for name in self.joint_hi:
            if segment not in name:
                continue
            best = max(best, self.joint_hi[name] - self.joint_lo[name])
        return math.degrees(best)

    def leg_travel_deg(self) -> float:
        return self.span_deg("femur")

    def stride_deg(self) -> float:
        return self.span_deg("coxa")

    def reset_joint_span(self) -> None:
        self.joint_lo.clear()
        self.joint_hi.clear()
        self.joint_sum.clear()
        self.joint_n.clear()
        self.ref_sum.clear()
        self.fb_sum.clear()
        self.ref_n.clear()

    # ------------------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        self.latest = msg
        if not self.recording:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((t, p.x, p.y, p.z, roll, pitch))

    # ------------------------------------------------------------------
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds: float, publish: Twist | None = None) -> None:
        """Spin for `seconds` of SIMULATION time, optionally commanding."""
        start = self.sim_now()
        # A wall-clock backstop: if sim time is frozen this would otherwise
        # never return, and the run would hang instead of reporting why.
        import time as _time
        wall_start = _time.monotonic()
        wall_limit = seconds * 20.0 + 30.0

        while self.sim_now() - start < seconds:
            if publish is not None:
                self.cmd_pub.publish(publish)
            rclpy.spin_once(self, timeout_sec=0.05)
            if _time.monotonic() - wall_start > wall_limit:
                self.get_logger().error(
                    "Simulation time is not advancing. Is Gazebo paused or dead?"
                )
                raise SystemExit(3)

    def wait_for_odom(self, timeout: float = 60.0) -> None:
        import time as _time
        deadline = _time.monotonic() + timeout
        while self.latest is None:
            if _time.monotonic() > deadline:
                self.get_logger().error(
                    f"No /odom in {timeout:.0f}s. The simulation is not "
                    "publishing odometry -- check that the sim launched."
                )
                raise SystemExit(2)
            rclpy.spin_once(self, timeout_sec=0.2)

    # ------------------------------------------------------------------
    def segment_stats(self, seg: list) -> dict:
        if len(seg) < 2:
            return {}
        t0, x0, y0, z0, _, _ = seg[0]
        t1, x1, y1, z1, _, _ = seg[-1]
        dt = max(t1 - t0, 1e-6)
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        zs = [s[3] for s in seg]
        tilts = [max(abs(s[4]), abs(s[5])) for s in seg]
        return {
            "dt": dt,
            "dx": dx,
            "dy": dy,
            "dist": dist,
            "speed": dist / dt,
            "fwd_speed": dx / dt,
            "z_mean": sum(zs) / len(zs),
            "z_min": min(zs),
            "z_max": max(zs),
            "tilt_max_deg": math.degrees(max(tilts)),
            "n": len(seg),
        }

    # ------------------------------------------------------------------
    def run(self) -> int:
        print("Waiting for /odom ...", flush=True)
        self.wait_for_odom()
        print("Odometry present. Starting measurement.\n", flush=True)

        stop = Twist()
        fwd = Twist()
        fwd.linear.x = self.speed

        self.recording = True

        mark0 = len(self.samples)
        print(f"[1/3] Baseline, {self.baseline_s:.0f}s, NO command ...", flush=True)
        self.reset_joint_span()
        self.spin_for(self.baseline_s, publish=stop)
        mark1 = len(self.samples)
        legs_idle = self.leg_travel_deg()

        print(f"[2/3] Driving forward at {self.speed:.3f} m/s for "
              f"{self.drive_s:.0f}s ...", flush=True)
        self.reset_joint_span()
        self.spin_for(self.drive_s, publish=fwd)
        mark2 = len(self.samples)
        legs_walk = self.leg_travel_deg()
        stride_walk = self.stride_deg()

        print(f"[3/3] Settling, {self.settle_s:.0f}s ...", flush=True)
        self.reset_joint_span()
        self.spin_for(self.settle_s, publish=stop)
        mark3 = len(self.samples)
        held = self.mean_by_segment()
        ctrl = self.ctrl_mean_by_segment()

        self.recording = False

        base = self.segment_stats(self.samples[mark0:mark1])
        walk = self.segment_stats(self.samples[mark1:mark2])
        rest = self.segment_stats(self.samples[mark2:mark3])

        if not base or not walk:
            print("\nNot enough odometry samples to score this run.")
            return 2

        expected = self.speed * walk["dt"]
        efficiency = walk["dx"] / expected if expected > 1e-9 else 0.0

        print("\n" + "=" * 66)
        print(" RESULT")
        print("=" * 66)
        print(f"{'':22}{'STANDING':>14}{'WALKING':>14}{'AFTER':>14}")
        print(f"{'duration (s)':22}{base['dt']:>14.1f}{walk['dt']:>14.1f}"
              f"{rest.get('dt', 0):>14.1f}")
        print(f"{'travel x (m)':22}{base['dx']:>14.4f}{walk['dx']:>14.4f}"
              f"{rest.get('dx', 0):>14.4f}")
        print(f"{'travel y (m)':22}{base['dy']:>14.4f}{walk['dy']:>14.4f}"
              f"{rest.get('dy', 0):>14.4f}")
        print(f"{'speed (m/s)':22}{base['speed']:>14.4f}{walk['speed']:>14.4f}"
              f"{rest.get('speed', 0):>14.4f}")
        print(f"{'body height (m)':22}{base['z_mean']:>14.4f}"
              f"{walk['z_mean']:>14.4f}{rest.get('z_mean', 0):>14.4f}")
        print(f"{'max tilt (deg)':22}{base['tilt_max_deg']:>14.1f}"
              f"{walk['tilt_max_deg']:>14.1f}"
              f"{rest.get('tilt_max_deg', 0):>14.1f}")
        print("-" * 66)
        print(f"femur swing idle     {legs_idle:.1f} deg peak-to-peak  (lift)")
        print(f"femur swing walking  {legs_walk:.1f} deg peak-to-peak  (lift)")
        print(f"COXA swing walking   {stride_walk:.1f} deg peak-to-peak  (STRIDE)")
        print(f"commanded speed      {self.speed:.4f} m/s")
        print(f"achieved speed       {walk['fwd_speed']:.4f} m/s")
        print(f"tracking efficiency  {efficiency * 100:.0f} %")
        print("-" * 66)
        print(" JOINT HOLD, at rest after walking  (mean over all six legs)")
        print(f"{'':10}{'measured':>12}{'design':>12}{'error':>12}{'error':>10}")
        for seg in ("coxa", "femur", "tibia"):
            if seg not in held:
                continue
            meas = held[seg]
            des = self.DESIGN[seg]
            err = meas - des
            print(f"{seg:10}{meas:>12.4f}{des:>12.4f}{err:>12.4f}"
                  f"{math.degrees(err):>9.1f}d")

        if ctrl:
            print("-" * 66)
            print(" WHAT THE CONTROLLER WAS ASKED FOR vs WHAT IT ACHIEVED")
            print(f"{'':10}{'asked':>12}{'achieved':>12}{'shortfall':>12}"
                  f"{'':>10}")
            for seg in ("coxa", "femur", "tibia"):
                if seg not in ctrl:
                    continue
                ref, fb = ctrl[seg]
                gap = fb - ref
                print(f"{seg:10}{ref:>12.4f}{fb:>12.4f}{gap:>12.4f}"
                      f"{math.degrees(gap):>9.1f}d")
            print("  asked == design  -> gait is right, controller is failing")
            print("  asked != design  -> gait is commanding the wrong pose")
        print("=" * 66)

        # The baseline window has been unreliable: if too little simulated
        # time elapsed in it, dividing displacement by that tiny dt reports a
        # huge fake drift speed. Say so rather than judging on it.
        if base["dt"] < 1.0:
            print(f"\nNOTE: baseline captured only {base['dt']:.2f}s of "
                  "simulated time, so the STANDING column is not meaningful "
                  "and is excluded from the verdict.")

        # ---- verdict ----------------------------------------------------
        # Ordered most-severe first: a robot that has been ejected also looks
        # like it travelled a long way, so ejection must be tested before
        # success or it will be reported as an excellent walk.
        verdict, detail, code = self.judge(
            base, walk, efficiency, legs_walk, stride_walk
        )
        print(f"\nVERDICT: {verdict}")
        print(detail)
        return code

    # ------------------------------------------------------------------
    def judge(self, base: dict, walk: dict, efficiency: float,
              legs_walk: float = -1.0, stride_walk: float = -1.0):
        # STRIDE FIRST. A hexapod propels itself with the coxa joints; if
        # they are not swinging there is no propulsion to evaluate, and any
        # conclusion about grip or gait quality would be about a robot that
        # was never asked to move.
        if 0.0 <= stride_walk < 2.0 and legs_walk >= 2.0:
            return ("STEPPING, NOT REACHING",
                    f"  Femur (lift) swung {legs_walk:.1f} deg, but coxa "
                    f"(stride) only {stride_walk:.1f} deg.\n"
                    "  The gait clock is running but the commanded VELOCITY is\n"
                    "  zero, so the legs step on the spot. Look at /cmd_vel\n"
                    "  reaching the gait node, and at the command watchdog.\n"
                    "  This is NOT a traction problem.", 16)

        # Check the legs FIRST. If they are not moving, every downstream
        # conclusion about grip, slip and gait is meaningless -- there is no
        # gait to judge. The previous version reported "marching in place",
        # which quietly assumed the legs were cycling.
        if 0.0 <= legs_walk < 2.0:
            return ("LEGS NOT MOVING",
                    f"  Largest femur swing during the drive phase was only\n"
                    f"  {legs_walk:.1f} deg. The robot is not attempting to walk\n"
                    "  at all. Either /cmd_vel is not reaching the gait node,\n"
                    "  the command watchdog is zeroing it, or the gait clock is\n"
                    "  not advancing. This is NOT a traction problem.", 15)

        if walk["z_mean"] > 0.45 or walk["tilt_max_deg"] > 60.0:
            return ("EJECTED / TIPPED",
                    "  The body left its normal height or rolled over. This is\n"
                    "  not walking -- energy is being injected somewhere.", 10)

        if walk["speed"] > self.speed * 6.0:
            return ("EJECTED (launched)",
                    f"  Travelled {walk['speed']:.2f} m/s against a commanded\n"
                    f"  {self.speed:.2f} m/s. That is a launch, not a walk.", 10)

        # Only judge drift if the baseline actually captured enough simulated
        # time to mean anything. A 0.1 s window turns 3 mm of settling into a
        # reported 200 mm/s, which sent us chasing a drift that was not real.
        if base["dt"] >= 1.0 and base["speed"] > 0.005:
            return ("DRIFTING WHILE IDLE",
                    f"  Moved {base['speed'] * 1000:.1f} mm/s with NO command.\n"
                    "  Fix the drift before judging the walk -- any travel\n"
                    "  measured while walking is contaminated by it.", 11)

        if efficiency > 0.6:
            return ("WALKING",
                    f"  Tracking {efficiency * 100:.0f} % of commanded speed,\n"
                    "  body height steady, staying upright. This works.", 0)

        if efficiency > 0.2:
            return ("WALKING, SLIPPING",
                    f"  Moving, but only {efficiency * 100:.0f} % of commanded\n"
                    "  speed. Feet are sliding during stance. Raise foot_mu, or\n"
                    "  shorten the stride.", 12)

        if walk["dist"] < 0.02:
            return ("MARCHING IN PLACE",
                    "  Legs cycling, body not travelling. Either the feet have\n"
                    "  no grip, or the stance phase is not pushing backwards.", 13)

        return ("WANDERING",
                f"  Net travel {walk['dist']:.3f} m but only "
                f"{efficiency * 100:.0f} % in the commanded direction.\n"
                "  Motion is not coordinated into a gait.", 14)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.03)
    ap.add_argument("--baseline", type=float, default=8.0)
    ap.add_argument("--drive", type=float, default=20.0)
    ap.add_argument("--settle", type=float, default=3.0)
    args = ap.parse_args()

    rclpy.init()
    node = WalkMeasurer(args.speed, args.baseline, args.drive, args.settle)
    try:
        return node.run()
    except SystemExit as exc:
        return int(exc.code or 1)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
