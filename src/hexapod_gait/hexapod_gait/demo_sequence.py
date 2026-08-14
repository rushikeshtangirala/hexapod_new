#!/usr/bin/env python3
"""
demo_sequence.py : drive the review demonstration from a script.

=============================================================================
WHY NOT JUST USE THE KEYBOARD
=============================================================================
Because a live demonstration has one job, which is to work, and
teleop_twist_keyboard adds three ways for it not to:

  1. The keyboard node only sends while its terminal has focus. Click on the
     Gazebo window to point at something and the robot silently stops.
  2. Its speed starts at 0.5 m/s, six times our envelope. Every command is
     clamped, so what you press is not what the robot does, and pressing a
     key twice does nothing visible.
  3. You have to narrate and type at the same time, in front of examiners,
     while watching whether it worked.

A scripted sequence removes all three. It runs the same manoeuvres every
time, in the same order, at speeds inside the verified envelope, and you can
talk over it. If you want the keyboard as well, both can run: this only
publishes when it is running.

=============================================================================
WHAT IT DEMONSTRATES, AND IN WHICH ORDER
=============================================================================
The order is chosen so each segment shows something the previous one could
not, which is what turns a demo into an argument:

  1. stand        the stance pose holds, nothing drifts
  2. forward      the tripod gait, and body travel
  3. stop         the settle sequence: legs finish the cycle and land
  4. turn left    per-leg stride differs; turning is the SAME code path
  5. forward      heading has actually changed
  6. turn right   symmetry, so nobody thinks one direction was a fluke
  7. strafe       sideways with no rotation, which a wheeled robot cannot do
  8. arc          linear and angular together, the general case
  9. reverse      the gait is not direction-special-cased
 10. stop         ends where it is stable, ready to run again

Speeds stay inside the envelope checked in demo.launch.py, so no stride is
ever clamped and the feet do not slide.

USAGE
    ros2 run hexapod_gait demo_sequence
    ros2 run hexapod_gait demo_sequence --ros-args -p loop:=true
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# (label, seconds, vx, vy, wz). Keep total near 75 s: long enough to show
# everything, short enough that nobody's attention wanders.
SEQUENCE = [
    ("standing still",           5.0,  0.00, 0.00,  0.00),
    ("walking forward",         12.0,  0.07, 0.00,  0.00),
    ("stopping, legs settle",    5.0,  0.00, 0.00,  0.00),
    ("turning left",             8.0,  0.00, 0.00,  0.25),
    ("forward on new heading",   8.0,  0.07, 0.00,  0.00),
    ("turning right",            8.0,  0.00, 0.00, -0.25),
    ("strafing right",           8.0,  0.00, -0.06, 0.00),
    ("arc: forward + turn",     10.0,  0.05, 0.00,  0.20),
    ("walking backward",         8.0, -0.06, 0.00,  0.00),
    ("stop",                     6.0,  0.00, 0.00,  0.00),
]


class DemoSequence(Node):

    def __init__(self) -> None:
        super().__init__("hexapod_demo_sequence")
        self.declare_parameter("loop", False)
        self.loop = bool(self.get_parameter("loop").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.step = 0
        self.elapsed = 0.0
        self.dt = 0.1
        self.announced = False

        total = sum(s[1] for s in SEQUENCE)
        self.get_logger().info(
            f"Demo sequence: {len(SEQUENCE)} segments, {total:.0f} s total."
            + ("  Looping." if self.loop else "")
        )
        self.create_timer(self.dt, self.on_timer)

    def on_timer(self) -> None:
        if self.step >= len(SEQUENCE):
            if self.loop:
                self.step = 0
                self.elapsed = 0.0
                self.announced = False
            else:
                self.pub.publish(Twist())          # explicit stop
                self.get_logger().info("Sequence complete.")
                raise SystemExit(0)
            return

        label, dur, vx, vy, wz = SEQUENCE[self.step]

        if not self.announced:
            self.announced = True
            # Numbered and timed, so you know what is coming and can narrate
            # ahead of it rather than describing what already happened.
            self.get_logger().info(
                f"[{self.step + 1}/{len(SEQUENCE)}] {label}  "
                f"({dur:.0f}s, vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f})"
            )

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub.publish(msg)

        self.elapsed += self.dt
        if self.elapsed >= dur:
            self.step += 1
            self.elapsed = 0.0
            self.announced = False


def main() -> None:
    rclpy.init()
    node = DemoSequence()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
