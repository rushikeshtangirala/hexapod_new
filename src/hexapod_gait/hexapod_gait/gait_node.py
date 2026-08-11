#!/usr/bin/env python3
"""
gait_node.py : the ROS wrapper around GaitGenerator.

DESIGN NOTE, AND IT IS THE POINT OF THE WHOLE PACKAGE
=====================================================
This file contains no kinematics and no gait mathematics. It subscribes,
ticks a clock, and publishes. Everything interesting lives in kinematics.py
and gait.py, which have no ROS dependency and are verified offline by
tools/verify_ik.py and tools/verify_gait.py.

That split is deliberate. Middleware is the least testable part of a robotics
stack: to exercise it you need a running graph, a simulator, timing, and
transport that all work. Mathematics needs none of those. Pushing every
decision out of the node and into plain functions is what let the gait be
proven correct before Gazebo was even reliable.

TOPICS
    subscribe  /cmd_vel                              geometry_msgs/Twist
    publish    /leg_position_controller/commands     std_msgs/Float64MultiArray

Using /cmd_vel means the standard teleop_twist_keyboard node drives this
directly, with no custom teleop to write or debug:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

WHY A FIXED-RATE TIMER RATHER THAN PUBLISHING ON EACH cmd_vel
=============================================================
The gait must keep stepping while the velocity command is unchanged. A
keyboard teleop publishes only on keypress, so driving the gait from the
subscription callback would make the robot advance one control step per
keystroke. The timer owns the clock; the subscription only updates the
target velocity. This is the normal shape for any periodic controller.

WATCHDOG
========
If no command arrives for command_timeout seconds the robot stops. A legged
robot that keeps walking after its operator disconnects is a hazard on
hardware and a nuisance in simulation.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait import GaitGenerator, GaitParams, check_reachable
from .kinematics import LEGS, UnreachableTarget

# Joint names in the order fixed by hexapod_control/config/controllers.yaml.
# Derived from LEGS rather than written out, so the ordering contract lives
# in exactly one place.
JOINT_NAMES: list[str] = [
    f"{leg.name}_{segment}_joint"
    for leg in LEGS
    for segment in ("coxa", "femur", "tibia")
]


class GaitNode(Node):

    def __init__(self) -> None:
        super().__init__("hexapod_gait_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("cycle_time", 1.4)
        self.declare_parameter("step_height", 0.045)
        self.declare_parameter("stance_radius", 0.34)
        self.declare_parameter("stance_height", -0.11)
        self.declare_parameter("max_stride", 0.10)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.50)
        self.declare_parameter("command_timeout", 1.0)

        gp = self.get_parameter
        self.rate = float(gp("control_rate").value)
        self.max_lin = float(gp("max_linear_speed").value)
        self.max_ang = float(gp("max_angular_speed").value)
        self.timeout = float(gp("command_timeout").value)

        params = GaitParams(
            cycle_time=float(gp("cycle_time").value),
            step_height=float(gp("step_height").value),
            stance_radius=float(gp("stance_radius").value),
            stance_height=float(gp("stance_height").value),
            max_stride=float(gp("max_stride").value),
        )
        self.gen = GaitGenerator(params)

        # ---- startup validation ------------------------------------------
        # Check the corners of the velocity envelope BEFORE accepting any
        # command. Discovering an unreachable target mid-demonstration is a
        # much worse outcome than refusing to start.
        self.get_logger().info("Validating velocity envelope...")
        bad = 0
        for label, vx, vy, wz in (
            ("max forward", self.max_lin, 0.0, 0.0),
            ("max reverse", -self.max_lin, 0.0, 0.0),
            ("max strafe", 0.0, self.max_lin, 0.0),
            ("max yaw", 0.0, 0.0, self.max_ang),
            ("combined", self.max_lin * 0.7, 0.0, self.max_ang * 0.7),
        ):
            problems = check_reachable(params, vx, vy, wz, samples=60)
            if problems:
                bad += 1
                self.get_logger().error(
                    f"  {label}: {len(problems)} unreachable, "
                    f"first: {problems[0]}"
                )
            else:
                self.get_logger().info(f"  {label}: ok")
        if bad:
            self.get_logger().error(
                "Velocity envelope contains unreachable targets. Reduce "
                "max_linear_speed / max_angular_speed, or raise "
                "stance_height, before walking."
            )

        # ---- ROS interfaces ----------------------------------------------
        # ---- output mode --------------------------------------------------
        # "trajectory" : JointTrajectory to joint_trajectory_controller,
        #                which closes a position loop over the EFFORT
        #                interface. Required for the free-base robot.
        # "position"   : Float64MultiArray to a forward position controller.
        #                Only usable with fix_base:=true.
        #
        # Must match control_mode in the launch. Mismatched, the node
        # publishes to a topic nobody is subscribed to and the robot simply
        # does not move, with no error anywhere.
        # Both control modes now use JointTrajectoryController, so both take
        # JointTrajectory messages. Only the controller NAME differs, hence a
        # topic parameter rather than a message-type switch.
        #
        # The old Float64MultiArray path is kept but is no longer the
        # default: a ForwardCommandController starts with a ZERO command
        # buffer, and with a position interface that zero is written to the
        # hardware as a real target on the first cycle. On a legged robot
        # that teleports every joint out of its stance and throws the robot
        # across the world. JTC holds the measured position on activation
        # instead, so there is no zero to jump to.
        self.declare_parameter("command_type", "trajectory")
        self.declare_parameter(
            "command_topic", "/leg_position_controller/joint_trajectory"
        )
        self.command_type = str(self.get_parameter("command_type").value)
        topic = str(self.get_parameter("command_topic").value)

        if self.command_type == "trajectory":
            self.traj_pub = self.create_publisher(JointTrajectory, topic, 10)
            self.pub = None
            self.get_logger().info(f"Publishing JointTrajectory on {topic}")
        else:
            self.pub = self.create_publisher(
                Float64MultiArray, "/leg_position_controller/commands", 10
            )
            self.traj_pub = None

        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd_vel, 10
        )

        # ---- startup ramp -------------------------------------------------
        # NEVER command a large instantaneous joint change while the feet are
        # in contact.
        #
        # gazebo_ros2_control realises a position command with SetPosition(),
        # which teleports the joint. A teleport in free space is harmless. A
        # teleport of a loaded leg drives the foot through the ground plane
        # in one physics step, and ODE removes that penetration with an
        # impulse large enough to throw the robot out of the world. This is
        # the failure we saw: "the robot stands, then moves away randomly".
        #
        # The launch file now spawns the robot already in the stance pose, so
        # the common case has no jump. This ramp covers the rest: restarting
        # the gait node against an already-running simulation, or a robot
        # left in some other pose. It blends from wherever the joints
        # actually are to the gait output over ramp_time.
        #
        # Note this is not a filter on the gait. Once the ramp completes it
        # has no effect at all; the gait's own trajectories are continuous by
        # construction and need no smoothing.
        self.declare_parameter("startup_ramp", 2.0)
        self.ramp_time = float(self.get_parameter("startup_ramp").value)
        self.initial_joints: list[float] | None = None
        self.ramp_elapsed = 0.0

        # ---- diagnostic mode ---------------------------------------------
        # hold_only:=true publishes the STANCE POSE and nothing else. The
        # gait clock never advances, velocities are always zero, cmd_vel is
        # ignored.
        #
        # This is a bisect, not a feature. If the robot is stable standing
        # but is ejected the moment the gait node starts, exactly one of two
        # things is true:
        #
        #   a) the commanded pose differs from where the robot actually is,
        #      so the first message is a large step -> hold_only ALSO ejects
        #   b) the gait's motion is the problem -> hold_only is stable
        #
        # They need opposite fixes, and no amount of watching Gazebo
        # distinguishes them.
        self.declare_parameter("hold_only", False)
        self.hold_only = bool(self.get_parameter("hold_only").value)
        if self.hold_only:
            self.get_logger().warn(
                "HOLD_ONLY: publishing the stance pose only. The gait clock "
                "will not advance and cmd_vel is ignored."
            )

        self.logged_first = False

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10
        )

        # ---- BLOCK until we know where the robot actually is ---------------
        #
        # THIS IS THE BUG THAT EJECTED THE ROBOT, AND IT IS WORTH SPELLING OUT.
        #
        # Previously the timer was created here, immediately. rclpy.spin() then
        # fired it 20 ms later. But a brand-new subscription does not receive
        # anything for as long as DDS discovery takes to match it to the
        # publisher -- typically 100 ms to 2 s, never 20 ms.
        #
        # So on the first tick self.initial_joints was None, the startup ramp
        # block below was skipped entirely, and the node published the RAW
        # STANCE POSE as its very first command. Over a position interface
        # Gazebo executes that by teleporting all 18 joints there in a single
        # physics step, from wherever the robot really was. Feet loaded, legs
        # snapped, robot launched.
        #
        # The old code even DETECTED this and logged an error about it -- and
        # then published the dangerous command anyway. Detecting a fatal
        # condition and proceeding regardless is worse than not detecting it.
        #
        # The ramp is the entire safety mechanism of this node. It cannot
        # function without a start pose, so there is no correct behaviour that
        # involves running without one. We wait, and if it never arrives we
        # refuse to publish at all.
        #
        # Wall time, not sim time, is deliberate: if /clock is not advancing
        # (Gazebo paused, or crashed) a sim-time deadline never expires and we
        # would hang here forever instead of reporting the real problem.
        self._wait_for_joint_states(timeout=20.0)

        self.last_cmd_time = self.get_clock().now()
        self.last_tick = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)

        if not self.get_parameter("use_sim_time").value:
            self.get_logger().warn(
                "use_sim_time is FALSE. If you are running against Gazebo, "
                "the gait clock will advance in wall time while the robot "
                "moves in simulation time. When Gazebo runs below 1.0 real "
                "time factor the legs will cycle faster than the body can "
                "travel, the feet will skid, and the walk will look random. "
                "Launch with:  -p use_sim_time:=true"
            )

        self.get_logger().info(
            f"Gait node up. {self.rate:.0f} Hz, cycle {params.cycle_time}s. "
            f"Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
        )

    # ----------------------------------------------------------------------
    class StartPoseUnavailable(RuntimeError):
        """No usable /joint_states arrived. Publishing would be unsafe."""

    def _wait_for_joint_states(self, timeout: float) -> None:
        """Spin until on_joint_states() has captured a full 18-joint pose.

        Raises StartPoseUnavailable rather than returning without one. The
        caller must not publish in that case -- a command issued from an
        unknown start pose is a step command, and a step command over a
        position interface is a teleport.
        """
        self.get_logger().info("Waiting for /joint_states before commanding...")

        deadline = time.monotonic() + timeout
        nagged = False

        while self.initial_joints is None:
            if time.monotonic() >= deadline:
                raise GaitNode.StartPoseUnavailable(
                    f"No complete /joint_states in {timeout:.0f}s. "
                    "Refusing to publish: without a start pose the first "
                    "command would be a step, which teleports the joints and "
                    "ejects the robot.\n"
                    "  Check:  ros2 control list_controllers\n"
                    "          joint_state_broadcaster must be 'active'\n"
                    "  Check:  ros2 topic hz /joint_states\n"
                    "  If Gazebo is paused, physics never steps and no joint "
                    "states are published -- unpause it."
                )

            # 5 s in, say something useful rather than sitting silent.
            if not nagged and time.monotonic() > deadline - timeout + 5.0:
                nagged = True
                self.get_logger().warn(
                    "Still no /joint_states after 5s. Is "
                    "joint_state_broadcaster active, and is Gazebo unpaused?"
                )

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Start pose acquired. Safe to command.")

    # ----------------------------------------------------------------------
    def on_joint_states(self, msg: JointState) -> None:
        """Capture the pose we are starting FROM, once."""
        if self.initial_joints is not None:
            return
        lookup = dict(zip(msg.name, msg.position))
        if not all(n in lookup for n in JOINT_NAMES):
            return                                  # partial message, wait
        self.initial_joints = [lookup[n] for n in JOINT_NAMES]
        self.get_logger().info(
            f"Captured start pose, ramping to stance over {self.ramp_time:.1f}s"
        )

    # ----------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist) -> None:
        """Clamp and store. No gait work happens here; see the module note."""
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self.gen.set_command(vx, vy, wz)
        self.last_cmd_time = self.get_clock().now()

    # ----------------------------------------------------------------------
    def on_timer(self) -> None:
        now = self.get_clock().now()

        # MEASURE dt, DO NOT ASSUME IT.
        #
        # The obvious version of this line is `self.gen.advance(1.0/self.rate)`,
        # which assumes every timer tick represents exactly one control
        # period. That assumption fails in the one environment that matters:
        # against Gazebo at a real time factor below 1.0, a 50 Hz wall-clock
        # timer fires 50 times per WALL second, but only (50 * RTF) times per
        # SIMULATED second. Advancing the gait phase by a fixed 20 ms each
        # time makes the gait run 1/RTF times too fast relative to the body.
        #
        # The non-slip guarantee derived in gait.py is stated in terms of the
        # body's velocity through the world. If the phase clock and the
        # physics clock disagree, that guarantee is void: the stance foot no
        # longer travels the distance the body travels, so it drags, and the
        # walk degenerates into skidding that looks like random motion.
        #
        # Reading the actual elapsed time from the node's clock makes this
        # correct under sim time and wall time alike, at any real time factor.
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now

        # Guard against a nonsensical delta: the first tick, a clock jump at
        # Gazebo reset, or a stall. Fall back to the nominal period rather
        # than teleporting the gait phase.
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate

        # Watchdog
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout or self.hold_only:
            self.gen.set_command(0.0, 0.0, 0.0)

        if not self.hold_only:
            self.gen.advance(dt)

        try:
            angles = self.gen.joint_array()
        except UnreachableTarget as exc:
            # Hold the previous command rather than publishing garbage. This
            # should be unreachable given the startup validation, so it is
            # logged loudly if it ever fires.
            self.get_logger().error(f"IK failed, holding position: {exc}")
            return

        # Blend out of the captured start pose. Cosine easing rather than a
        # straight line so the ramp begins and ends with zero velocity; a
        # linear blend steps the joint velocity at both ends, which is the
        # smaller version of the same discontinuity we are avoiding.
        if self.initial_joints is not None and self.ramp_elapsed < self.ramp_time:
            self.ramp_elapsed += dt
            t = min(1.0, self.ramp_elapsed / max(self.ramp_time, 1e-6))
            blend = 0.5 - 0.5 * math.cos(math.pi * t)
            angles = [
                start + (target - start) * blend
                for start, target in zip(self.initial_joints, angles)
            ]
            if t >= 1.0:
                self.get_logger().info("Ramp complete, gait active.")

        # Log the first command against the measured joint state. If the
        # robot is ejected the moment this node starts, the answer is almost
        # always visible in this one table: a large delta means the first
        # message is a step command, which a position interface executes by
        # teleporting the joint.
        if not self.logged_first:
            self.logged_first = True
            if self.initial_joints is None:
                # Unreachable: __init__ refuses to create this timer until a
                # start pose exists. Kept as an assertion because the failure
                # mode it guards against is destructive rather than merely
                # wrong -- if this ever fires, stop, do not publish.
                self.get_logger().fatal(
                    "initial_joints is None inside on_timer. This should be "
                    "impossible. Not publishing."
                )
                return
            else:
                # Compare the measured pose against the RAW GAIT TARGET, not
                # against the ramped command. The ramp starts from the
                # measured pose by construction, so comparing to it always
                # shows zero and tells you nothing. The earlier version of
                # this log did exactly that and was useless.
                target = self.gen.joint_array()
                worst = 0.0
                worst_name = ""
                lines = []
                for name, now_a, cmd_a in zip(
                    JOINT_NAMES, self.initial_joints, target
                ):
                    delta = cmd_a - now_a
                    if abs(delta) > abs(worst):
                        worst, worst_name = delta, name
                    lines.append(
                        f"  {name:<16} now {now_a:+.4f}  cmd {cmd_a:+.4f}  "
                        f"delta {delta:+.4f}"
                    )
                self.get_logger().info(
                    "MEASURED POSE vs GAIT STANCE TARGET\n" + "\n".join(lines)
                )
                self.get_logger().info(
                    f"largest delta: {worst:+.4f} rad "
                    f"({math.degrees(worst):+.2f} deg) on {worst_name}"
                )
                if abs(worst) > 0.05:
                    self.get_logger().error(
                        "That delta is large. With a position interface it "
                        "will be executed as a teleport, which ejects a "
                        "robot standing on loaded feet. The commanded stance "
                        "and the URDF initial_value do not agree."
                    )

        if self.command_type == "trajectory":
            self.publish_trajectory(angles)
        else:
            msg = Float64MultiArray()
            msg.data = [float(a) for a in angles]
            self.pub.publish(msg)

    # ----------------------------------------------------------------------
    def publish_trajectory(self, angles: list[float]) -> None:
        """
        Stream a single trajectory point.

        WHY ONE POINT AT A TIME
        JointTrajectoryController is normally given a whole trajectory once
        and left to execute it. A gait cannot work that way: it is
        regenerated continuously from a velocity command that the operator
        may change at any moment. So we publish one point per control cycle
        and each message replaces the last. This is the standard pattern for
        streaming a periodic controller through JTC.

        WHY time_from_start IS TWO CONTROL PERIODS
        Zero would ask for the position instantly, giving the interpolator
        nothing to work with and producing a step. Too large and the
        controller lags behind the gait. Two periods gives it a short,
        well-defined interval to interpolate across, which it will replace
        before reaching the end anyway.

        WHY VELOCITIES ARE INCLUDED
        Under effort control a pure PID must build up position error before
        it generates torque, so it always trails a moving target. Supplying
        the expected velocity as feedforward lets the controller anticipate
        rather than chase. On a swinging leg this is the difference between
        visibly lagging and tracking cleanly.
        """
        horizon = 2.0 / self.rate

        # During the startup ramp the commanded angles are a blend towards
        # the gait, not the gait itself, so the gait's own velocities do not
        # describe them. Publishing them anyway would tell the controller to
        # move faster than the ramp intends. Zero is the honest value here;
        # the ramp is slow enough that feedforward buys nothing.
        if self.initial_joints is not None and self.ramp_elapsed < self.ramp_time:
            velocities = [0.0] * len(angles)
        else:
            velocities = [float(v) for v in self.gen.joint_velocities()]

        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.velocities = velocities
        point.time_from_start = Duration(
            sec=int(horizon),
            nanosec=int((horizon % 1.0) * 1e9),
        )

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]
        self.traj_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    # GaitNode.__init__ blocks waiting for /joint_states and raises if it
    # never arrives. Exiting non-zero here is the correct outcome: the launch
    # file reports a failed node, which is loud and diagnosable. The
    # alternative -- starting anyway -- ends with the robot across the world.
    try:
        node = GaitNode()
    except GaitNode.StartPoseUnavailable as exc:
        print(f"\n[gait_node] FATAL: {exc}\n", flush=True)
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
