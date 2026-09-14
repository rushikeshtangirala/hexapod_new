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
            "spawn_height": LaunchConfiguration("spawn_height"),
            "gui": "true",
        }.items(),
    )

    # ------------------------------------------------------------------
    # control_mode is now forwarded EXPLICITLY, though it was already
    # arriving.
    #
    # An included launch file inherits the parent's launch configurations,
    # and a DeclareLaunchArgument does not override a configuration that is
    # already set. So walk.launch.py was seeing this file's control_mode
    # rather than its own "effort" default, which is why the anchored demo
    # correctly published to /leg_position_controller/joint_trajectory.
    #
    # It is written out anyway because relying on inheritance makes the two
    # files look independent when they are not: walk.launch.py's stated
    # default is "effort" and it has never once taken effect from here.
    # Passing it explicitly means the coupling is visible at the call site
    # instead of being a property of launch's scoping rules.
    # ------------------------------------------------------------------
    walk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup, "launch", "walk.launch.py"])
        ]),
        launch_arguments={
            "control_mode": LaunchConfiguration("control_mode"),
            "command_type": LaunchConfiguration("command_type"),
            "hold_only": LaunchConfiguration("hold_only"),
            "startup_ramp": LaunchConfiguration("startup_ramp"),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("control_mode", default_value="position"),
        DeclareLaunchArgument("fix_base", default_value="true"),
        DeclareLaunchArgument("command_type", default_value="trajectory"),

        # Free-base spawn height. 0.135 gives 3 mm of clearance: the foot
        # contact point is 0.132 below base_link, being the 0.120 stance
        # depth plus the 0.012 foot sphere. Ignored when fix_base is true,
        # because the weld then sets the height on its own.
        DeclareLaunchArgument("spawn_height", default_value="0.135"),

        # Bisection knobs, forwarded so the whole experiment is one command.
        # hold_only exercises the entire command path without ever cycling a
        # leg, which separates "the command path ejects it" from "the gait
        # motion ejects it". Those have nothing in common and no single fix
        # addresses both.
        DeclareLaunchArgument("hold_only", default_value="false"),
        DeclareLaunchArgument("startup_ramp", default_value="2.0"),

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
