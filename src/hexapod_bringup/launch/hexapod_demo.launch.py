#!/usr/bin/env python3
"""
hexapod_demo.launch.py : the whole demonstration, one command.

    ros2 launch hexapod_bringup hexapod_demo.launch.py

then, in a SECOND terminal:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -p speed:=0.06 -p turn:=0.3

Two terminals, no arguments, nothing to mistype.

WHAT IT STARTS
  gzserver + gzclient        physics and the Gazebo window
  robot_state_publisher      URDF and TF
  spawn_entity               inserts the robot, already in its stance pose
  joint_state_broadcaster    publishes /joint_states from the hardware
  leg_position_controller    accepts the 18 joint commands
  hexapod_gait_node          inverse kinematics and the tripod gait

WHY THE GAIT NODE STARTS ON A TIMER
The controller_manager does not exist until gzserver has loaded the plugin
and spawn_entity has inserted the robot, and the controllers are spawned
after that again. The gait node has nothing to publish to until all of that
has happened. hexapod_sim.launch.py chains its own stages on process exit
events, which is precise, but those events are not visible to an outer
launch file including it. A generous fixed delay is the honest solution
here: 15 seconds is far longer than the sequence needs, and the gait node
holds the robot in its stance pose when it does start anyway, so being late
costs nothing while being early costs everything.

WHY TELEOP IS NOT INCLUDED
teleop_twist_keyboard reads raw keypresses from its own terminal. Nodes
started by `ros2 launch` share the launcher's stdin, so it would start, see
no keyboard, and publish nothing. Any interactive node needs its own
terminal.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup = FindPackageShare("hexapod_bringup")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup, "launch", "hexapod_sim.launch.py"])
        ]),
        launch_arguments={
            "control_mode": LaunchConfiguration("control_mode"),
            "fix_base": LaunchConfiguration("fix_base"),
            "gui": "true",
        }.items(),
    )

    walk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup, "launch", "walk.launch.py"])
        ]),
        launch_arguments={
            "command_type": LaunchConfiguration("command_type"),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="position"),
        DeclareLaunchArgument("fix_base", default_value="true"),
        DeclareLaunchArgument("command_type", default_value="trajectory"),

        sim,

        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg="\n"
                            "============================================\n"
                            " Starting the gait node.\n"
                            " In ANOTHER terminal, run:\n"
                            "   ros2 run teleop_twist_keyboard "
                            "teleop_twist_keyboard \\\n"
                            "       --ros-args -p speed:=0.06 -p turn:=0.3\n"
                            " Then click that window and press  i\n"
                            "============================================"),
                walk,
            ],
        ),
    ])
