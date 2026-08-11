#!/usr/bin/env python3
"""
walk.launch.py : start the gait generator against the running simulation.

Run this AFTER hexapod_sim.launch.py, in a second terminal. Then drive the
robot from a third terminal with:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -p speed:=0.08 -p turn:=0.4

WHY TELEOP IS NOT IN THIS LAUNCH FILE
=====================================
teleop_twist_keyboard reads raw keypresses from its terminal. Nodes started
by `ros2 launch` do not get an interactive stdin: launch multiplexes their
output and takes the terminal for itself. The teleop node would start, see
no keyboard, and publish nothing. Any interactive node has to be run
directly from its own terminal.

WHY use_sim_time MATTERS HERE MORE THAN ANYWHERE ELSE
=====================================================
The gait's non-slip guarantee is a statement about distance travelled per
unit time. It only holds if the gait clock and the physics clock agree. With
use_sim_time false, a 50 Hz timer ticks 50 times per WALL second while
Gazebo may only simulate 0.3 seconds in that time, so the phase advances
over three times too fast relative to the body's actual motion. The stance
foot is then commanded to move further than the body moves, it drags along
the ground, and the robot skitters instead of walking.

This is not a subtle degradation. It is the difference between walking and
not walking, and it looks like a gait bug rather than a clock bug, which is
why it is worth stating twice.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # MUST match control_mode in hexapod_sim.launch.py.
        #   trajectory -> JointTrajectory to leg_trajectory_controller (effort)
        #   position   -> Float64MultiArray to leg_position_controller
        # Mismatched, the node publishes to a topic with no subscriber and
        # the robot silently does not move.
        # Both modes use JointTrajectoryController now, so both take
        # JointTrajectory. Only the controller name, and therefore the topic,
        # differs between them.
        DeclareLaunchArgument(
            "command_type", default_value="trajectory",
            description="trajectory (both modes) or position (legacy).",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/leg_position_controller/joint_trajectory",
            description="Use /leg_trajectory_controller/joint_trajectory "
                        "when running control_mode:=effort.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Take time from Gazebo's /clock. Set false only for "
                        "real hardware.",
        ),
        DeclareLaunchArgument(
            "cycle_time", default_value="1.4",
            description="Seconds per full gait cycle. Larger is slower and "
                        "more stable.",
        ),
        DeclareLaunchArgument(
            "step_height", default_value="0.045",
            description="Swing clearance above stance height, metres.",
        ),
        DeclareLaunchArgument(
            "max_linear_speed", default_value="0.10",
            description="Clamp on commanded linear speed, m/s. The node "
                        "validates the whole envelope at startup.",
        ),
        DeclareLaunchArgument(
            "max_angular_speed", default_value="0.50",
            description="Clamp on commanded yaw rate, rad/s.",
        ),
        DeclareLaunchArgument(
            "max_stride", default_value="0.10",
            description="Maximum foot displacement per stance, metres. "
                        "Lower values mean shorter, gentler steps.",
        ),
        DeclareLaunchArgument(
            "control_rate", default_value="50.0",
            description="Gait update rate, Hz.",
        ),
        DeclareLaunchArgument(
            "hold_only", default_value="false",
            description="Diagnostic: publish only the stance pose, never "
                        "advance the gait. Separates 'first command is a "
                        "step' from 'the gait motion is the problem'.",
        ),
        DeclareLaunchArgument(
            "startup_ramp", default_value="2.0",
            description="Seconds to blend from the measured pose to the "
                        "gait output.",
        ),
        DeclareLaunchArgument(
            "stance_radius", default_value="0.28",
            description="Nominal foot distance from the coxa axis, metres.",
        ),
        DeclareLaunchArgument(
            "stance_height", default_value="-0.18",
            description="Nominal foot depth below the coxa axis, metres.",
        ),

        Node(
            package="hexapod_gait",
            executable="gait_node",
            name="hexapod_gait_node",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "command_type": LaunchConfiguration("command_type"),
                "command_topic": LaunchConfiguration("command_topic"),
                "hold_only": LaunchConfiguration("hold_only"),
                "startup_ramp": LaunchConfiguration("startup_ramp"),
                "control_rate": LaunchConfiguration("control_rate"),
                "cycle_time": LaunchConfiguration("cycle_time"),
                "step_height": LaunchConfiguration("step_height"),
                "stance_radius": LaunchConfiguration("stance_radius"),
                "stance_height": LaunchConfiguration("stance_height"),
                "max_stride": LaunchConfiguration("max_stride"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_speed": LaunchConfiguration("max_angular_speed"),
                "command_timeout": 1.0,
            }],
        ),
    ])
