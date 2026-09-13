#!/usr/bin/env python3
"""
display.launch.py -- visualise the robot description in RViz2, no physics.

WHAT THIS LAUNCH FILE ACTUALLY DOES
-----------------------------------
It starts three nodes that together turn a static XML file into a live,
articulable 3D model:

  1. robot_state_publisher
       Reads the URDF from the `robot_description` parameter, subscribes to
       /joint_states, and publishes the resulting TF transforms. This node is
       the bridge between "a description of a robot" and "a coordinate frame
       tree you can query". It publishes:
         - /tf        for every MOVABLE joint (recomputed as joints move)
         - /tf_static for every FIXED joint (published once, latched)

  2. joint_state_publisher_gui
       A slider panel. Publishes /joint_states for every non-fixed joint.
       This is a STAND-IN for real feedback -- in Phase 5 it gets replaced by
       joint_state_broadcaster reading actual controller state. Running both
       at once is a classic mistake: two publishers fight over /joint_states
       and the model twitches between them.

  3. rviz2
       Subscribes to /tf and /robot_description and renders.

WHY WE VISUALISE BEFORE SIMULATING
----------------------------------
RViz shows KINEMATICS only -- no gravity, no contacts, no solver. If the model
is wrong here, it is wrong everywhere, but here it is wrong in a way you can
actually see and diagnose. Debugging a frame error inside a diverging Gazebo
physics step is dramatically harder. Always: RViz first, Gazebo second.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("hexapod_description")

    model_path = PathJoinSubstitution([pkg_share, "urdf", "hexapod.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "hexapod.rviz"])

    # ------------------------------------------------------------------
    # Run xacro at LAUNCH TIME rather than shipping a pre-generated .urdf.
    #
    # Why: a committed .urdf is a build artefact masquerading as source. It
    # goes stale the moment someone edits the xacro and forgets to regenerate,
    # and the resulting bug ("my change had no effect") is maddening.
    #
    # ParameterValue(..., value_type=str) is REQUIRED. Without it, the launch
    # system tries to infer the parameter type from the XML string, sees
    # something YAML-ish, and either mangles it or throws. This single missing
    # wrapper is one of the most common ROS 2 Humble launch failures.
    # ------------------------------------------------------------------
    # Mesh orientations are passed through so they can be tried WITHOUT
    # editing a file and rebuilding. Finding the right roll for a part is a
    # search over a handful of 90 degree options, and a search is only
    # practical if each trial is cheap:
    #
    #   ros2 launch hexapod_description display.launch.py \
    #        tibia_rpy:="1.5708 0 0"
    #
    # Once a value is right, write it into the default in
    # common_properties.xacro so it becomes the committed state.
    robot_description = ParameterValue(
        Command([
            "xacro ", model_path,
            " coxa_rpy:='", LaunchConfiguration("coxa_rpy"), "'",
            " femur_rpy:='", LaunchConfiguration("femur_rpy"), "'",
            " tibia_rpy:='", LaunchConfiguration("tibia_rpy"), "'",
            " tibia_nudge:='", LaunchConfiguration("tibia_nudge"), "'",
            " body_rpy:='", LaunchConfiguration("body_rpy"), "'",
            " enable_imu:=", LaunchConfiguration("enable_imu"),
            " enable_camera:=", LaunchConfiguration("enable_camera"),
            " enable_lidar:=", LaunchConfiguration("enable_lidar"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start joint_state_publisher_gui sliders.",
        ),
        DeclareLaunchArgument("coxa_rpy", default_value="0 0 0",
                              description="Coxa mesh roll pitch yaw, radians."),
        DeclareLaunchArgument("femur_rpy", default_value="0 0 0",
                              description="Femur mesh roll pitch yaw."),
        # These two defaults MUST match the xacro defaults in
        # common_properties.xacro. A DeclareLaunchArgument default is not a
        # fallback to the xacro value, it REPLACES it: the argument is always
        # passed on the xacro command line, so a stale "0 0 0" here silently
        # cancels whatever is committed in the model. That is why the tibia
        # flip appeared to do nothing when launched through this file even
        # after the xacro default had been changed.
        DeclareLaunchArgument("tibia_rpy", default_value="0 0 0",
                              description="Tibia mesh roll pitch yaw."),
        DeclareLaunchArgument("tibia_nudge", default_value="0 0 0",
                              description="Tibia mesh offset, metres. Must "
                                          "equal the mesh x span whenever "
                                          "tibia_rpy contains a 180 degree "
                                          "turn about Y."),
        # Sensors. Here they are all defaulted ON, unlike the simulation
        # launch. RViz only draws the LINKS and reads the TF tree; it never
        # renders an image or casts a ray, so the cost that makes them opt in
        # under Gazebo does not exist here. This is the cheapest way to check
        # that a mount pose is right before paying for the simulated sensor.
        DeclareLaunchArgument("enable_imu", default_value="true",
                              description="Show the IMU link."),
        DeclareLaunchArgument("enable_camera", default_value="true",
                              description="Show the camera links, including "
                                          "the optical frame."),
        DeclareLaunchArgument("enable_lidar", default_value="true",
                              description="Show the lidar link."),

        DeclareLaunchArgument("body_rpy", default_value="0 0 0",
                              description="Body mesh roll pitch yaw."),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                # Wall clock here. In Gazebo (Phase 4) this flips to true so
                # every node shares simulation time from /clock. Mixing the two
                # produces TF extrapolation errors that look like model bugs.
                "use_sim_time": False,
            }],
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
