#!/usr/bin/env python3
"""
demo.launch.py : the review demonstration. One command, robot walks.

===========================================================================
WHAT THIS IS
===========================================================================
    Gazebo, robot spawned, controllers up
  + hexapod_gait_node       real tripod gait, real inverse kinematics
  + hexapod_body_driver     body pose integrated from the gait

The legs are driven by the same verified mathematics as every other
configuration: kinematics.py and gait.py, checked offline by verify_ik.py
(round trip error 3.5e-16 m) and verify_gait.py (498 assertions including a
signed non-slip test). Nothing about the gait is special-cased for the demo.

What is different is that the BODY pose is integrated from the gait rather
than recovered by Gazebo's contact solver. The floating base on six frictional
contacts is the one part of this stack that has not been made to behave, and
it is not on the critical path for showing that the gait works.

===========================================================================
SAY THIS, IN THESE WORDS, IF ASKED
===========================================================================
"This is a kinematic simulation. The leg trajectories come from the gait
generator and the analytic IK. The body displacement is the displacement
those trajectories geometrically require, because the gait is provably
non-slip: stance feet have zero velocity relative to the ground at every
phase, which we assert in the offline verifier. The dynamic simulation is a
separate result, and I can tell you what we measured there."

That is accurate, and it is a stronger answer than a robot that walks with
no account of why the dynamic path did not.

DO NOT quote torque, contact force, slip or stability margin from this. Those
belong to sim_profile:=physical, and the measurements there are real:

    position + measured model    legs swing 10 deg, body travels 0.4 mm / 20 s
    effort   + hexapod_ros model walks at 99 percent speed tracking, but
                                 climbs 5 cm and tilts 12 degrees
    effort   + measured model    body pinned at the origin, unresolved

===========================================================================
USAGE
    ros2 launch hexapod_bringup demo.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard      (second terminal)

    i forward, k stop, j / l turn, comma reverse.
    Start with one or two taps. Hold nothing down for the first run.
===========================================================================
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare("hexapod_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        # Slower than the default 1.4 s. A longer cycle makes the leg motion
        # legible to someone watching for the first time, and this is a
        # demonstration, not a speed record.
        DeclareLaunchArgument("cycle_time", default_value="2.0"),
        DeclareLaunchArgument("step_height", default_value="0.045"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.08"),
        DeclareLaunchArgument("max_angular_speed", default_value="0.40"),

        # ------------------------------------------------------------------
        # 1. Simulation.
        #
        # control_mode:=position because config B measured it tracking
        # commanded joint angles to four decimal places. Its known weakness,
        # that position commands generate no propulsion, does not matter here:
        # propulsion is not what moves the body in this configuration.
        #
        # sim_profile:=demo keeps every measured mass and inertia and only
        # disables gravity, so the solver does not fight the imposed pose.
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bringup, "launch", "hexapod_sim.launch.py"])
            ]),
            launch_arguments={
                "control_mode": "position",
                "sim_profile": "demo",
                "fix_base": "false",
                "gui": LaunchConfiguration("gui"),
                "rviz": "false",
            }.items(),
        ),

        # ------------------------------------------------------------------
        # 2. Gait, once the controllers exist.
        #
        # The delay is not superstition: the gait node blocks until it has a
        # start pose from /joint_states, and starting it before the
        # controllers are active means it sits waiting while the robot stands
        # unpowered. 12 s is comfortable on a WSL2 machine at a real time
        # factor of 0.3 to 0.6.
        # ------------------------------------------------------------------
        TimerAction(period=12.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([bringup, "launch", "walk.launch.py"])
                ]),
                launch_arguments={
                    "control_mode": "position",
                    "cycle_time": LaunchConfiguration("cycle_time"),
                    "step_height": LaunchConfiguration("step_height"),
                    "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                    "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                    "startup_ramp": "3.0",
                }.items(),
            ),
        ]),

        # ------------------------------------------------------------------
        # 3. Body driver, after the gait has settled into stance.
        #
        # Started last so the robot is already standing correctly before its
        # pose is taken over. Starting it first would drive the body while the
        # legs were still ramping, and the feet would visibly skate.
        # ------------------------------------------------------------------
        TimerAction(period=17.0, actions=[
            Node(
                package="hexapod_gait",
                executable="body_driver",
                name="hexapod_body_driver",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "model_name": "hexapod",
                    "rate": 50.0,
                    "ride_height": 0.195,
                    "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                    "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                }],
            ),
        ]),
    ])
