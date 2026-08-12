#!/usr/bin/env python3
"""
body_driver.py : move the body according to the gait, instead of according to
                 the contact solver.

=============================================================================
WHY THIS EXISTS
=============================================================================
Every failure this project has had in Gazebo is the same failure: a FLOATING
BASE standing on six frictional contacts. It has appeared as marching in
place, as a body that climbs while walking, as a robot thrown across the
world, and as a base pinned at the origin. Measured, in three configurations:

    position + measured model   legs swing 10 deg, body travels 0.4 mm in 20 s
    effort   + hexapod_ros model  walks, but climbs 5 cm and tilts 12 degrees
    effort   + measured model   body pinned at (0,0,0), joints correct

The parts that are NOT at fault, and this matters:

  * the gait          verify_gait.py, 498 checks, including signed non-slip
  * the IK            verify_ik.py, round trip error 3.5e-16 m
  * the URDF          mount positions and joint axes match kinematics.py exactly
  * the controller    config B tracked commanded angles to four decimal places

So the mathematics is right and the contact integration is what nobody has
made behave. This node removes the contact integration from the demonstration
by driving the body pose directly.

=============================================================================
WHY THIS IS NOT CHEATING, AND HOW TO SAY SO
=============================================================================
The body displacement is NOT invented. The gait guarantees non-slip: every
stance foot has exactly zero velocity over the ground, asserted at every
sampled phase by verify_gait.py section 4. A foot planted on the ground while
the body advances therefore implies a body displacement, exactly, by
geometry. This node integrates precisely that implied displacement.

In other words: the legs move because of the real gait and the real inverse
kinematics; the body moves by the amount those legs geometrically require.
What is skipped is only the step where Gazebo rediscovers that displacement
by integrating friction cones.

Describe it as KINEMATIC SIMULATION and it is a completely standard and
honest thing to present. What would be dishonest is calling it a dynamic
result, or quoting a torque, contact force or slip figure from it. Do not.

The dynamic results are reported separately, and they are real findings:
position versus effort with numbers, the measured-versus-non-physical model
comparison, and the stance sign error that was found and fixed.

=============================================================================
HOW IT WORKS
=============================================================================
    /cmd_vel  ->  same filter and deadband the gait uses
              ->  integrate body pose in the world frame
              ->  /set_entity_state on model 'hexapod' at 50 Hz

The velocity is filtered with the SAME time constant as GaitGenerator, and
imported from GaitParams rather than copied, so the two cannot drift apart.
If they did, the body would advance at a different rate from the one the legs
imply and the feet would visibly skate: the exact artefact this node exists
to avoid.

Gravity should be disabled for the demo (sim_profile:=demo) so that physics
is not fighting the pose we impose. Without that, each set_entity_state is a
teleport that the solver then tries to correct, and the robot jitters.

USAGE
    ros2 launch hexapod_bringup demo.launch.py
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .gait import GaitParams


class BodyDriver(Node):

    def __init__(self) -> None:
        super().__init__("hexapod_body_driver")

        gp = GaitParams()

        self.declare_parameter("model_name", "hexapod")
        self.declare_parameter("rate", 50.0)
        # Ride height of base_link above the ground. Stance depth plus the
        # foot sphere radius: 0.180 + 0.015. If the feet visibly float or
        # sink, this is the number to adjust, and it is the ONLY number here
        # that is a presentation choice rather than a derived quantity.
        self.declare_parameter("ride_height", 0.195)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.50)
        # Imported, not typed in. See the module note.
        self.declare_parameter("command_tau", gp.command_tau)
        self.declare_parameter("velocity_deadband", gp.velocity_deadband)
        self.declare_parameter("stance_radius", gp.stance_radius)

        self.model = str(self.get_parameter("model_name").value)
        self.rate = float(self.get_parameter("rate").value)
        self.z = float(self.get_parameter("ride_height").value)
        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.max_ang = float(self.get_parameter("max_angular_speed").value)
        self.tau = float(self.get_parameter("command_tau").value)
        self.deadband = float(self.get_parameter("velocity_deadband").value)
        self.stance_radius = float(self.get_parameter("stance_radius").value)

        # World pose being integrated.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Commanded and filtered velocities, mirroring GaitGenerator.
        self.vx_cmd = self.vy_cmd = self.wz_cmd = 0.0
        self.vx = self.vy = self.wz = 0.0

        # ---- gazebo state service ----------------------------------------
        from gazebo_msgs.srv import SetEntityState
        self.cli = self.create_client(SetEntityState, "/set_entity_state")

        self.get_logger().info("Waiting for /set_entity_state ...")
        if not self.cli.wait_for_service(timeout_sec=30.0):
            # Fail loudly rather than sitting silently doing nothing, which
            # is the failure mode that cost this project several test cycles.
            self.get_logger().fatal(
                "/set_entity_state never appeared. libgazebo_ros_state.so is "
                "declared in hexapod_gazebo/worlds/hexapod.world; check that "
                "the world actually loaded and that you rebuilt after the "
                "world file changed."
            )
            raise SystemExit(2)
        self.get_logger().info("Connected. Driving the body from the gait.")

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self.last = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.on_timer)
        self._pending = None

    # ----------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist) -> None:
        self.vx_cmd = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        self.vy_cmd = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        self.wz_cmd = max(-self.max_ang, min(self.max_ang, msg.angular.z))

    # ----------------------------------------------------------------------
    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate

        # Same first-order lag as GaitGenerator._filter_command, so the body
        # accelerates exactly as fast as the legs think it does.
        if self.tau > 1e-9:
            a = 1.0 - math.exp(-dt / self.tau)
        else:
            a = 1.0
        self.vx += a * (self.vx_cmd - self.vx)
        self.vy += a * (self.vy_cmd - self.vy)
        self.wz += a * (self.wz_cmd - self.wz)

        # Same deadband the gait uses, so the body is stationary exactly when
        # the legs are.
        speed = math.hypot(self.vx, self.vy) + abs(self.wz) * self.stance_radius
        if speed >= self.deadband:
            # Body-frame velocity into the world frame. Standard planar
            # integration; yaw first so the step uses the mid-step heading.
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (self.vx * c - self.vy * s) * dt
            self.y += (self.vx * s + self.vy * c) * dt
            self.yaw += self.wz * dt

        self.publish_pose()

    # ----------------------------------------------------------------------
    def publish_pose(self) -> None:
        from gazebo_msgs.srv import SetEntityState

        # Do not queue a second request while one is outstanding. At 50 Hz an
        # unbounded queue of service calls will outrun Gazebo and the pose
        # falls behind the legs, which looks like skating.
        if self._pending is not None and not self._pending.done():
            return

        req = SetEntityState.Request()
        req.state.name = self.model
        req.state.pose.position.x = self.x
        req.state.pose.position.y = self.y
        req.state.pose.position.z = self.z
        req.state.pose.orientation.z = math.sin(self.yaw * 0.5)
        req.state.pose.orientation.w = math.cos(self.yaw * 0.5)
        req.state.reference_frame = "world"
        self._pending = self.cli.call_async(req)


def main() -> None:
    rclpy.init()
    node = BodyDriver()
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
