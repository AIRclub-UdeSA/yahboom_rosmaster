#!/usr/bin/env python3
"""
Launch the ROSMASTER X3 simulation on Gazebo Classic.

This is the macOS backend. Gazebo Fortress cannot open its GUI or run rendering
sensors on macOS: it initialises ogre2 on a secondary thread, and macOS only
allows window creation on the main thread. Gazebo Classic opens its GUI
normally, and its CPU raycast lidar bypasses rendering entirely, so /scan works.

Two macOS details are handled here rather than upstream:

* gazebo_ros ships gzserver.launch.py, but it hardcodes ".so" when naming its
  system plugins, while a macOS build installs ".dylib". Using it means gzserver
  starts without libgazebo_ros_factory, and /spawn_entity never appears. So
  gzserver and gzclient are started directly with the right extension.
* The robot is spawned with spawn_entity.py rather than a pre-converted SDF.
  Gazebo's own URDF parser resolves the joint chain, whereas `gz sdf -p` emits
  SDF 1.7 relative_to poses that Classic leaves unresolved, collapsing all four
  wheels onto the model origin.
"""
import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Gazebo plugin suffix. conda builds install .dylib on macOS; Linux uses .so.
PLUGIN_EXT = ".dylib" if platform.system() == "Darwin" else ".so"


def _robot_description(context):
    """Expand the xacro with the Gazebo Classic plugin set."""
    pkg_desc = get_package_share_directory("yahboom_rosmaster_description")
    xacro_path = os.path.join(pkg_desc, "urdf", "robots", "rosmaster_x3.urdf.xacro")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    description = Command([
        "xacro ", xacro_path,
        " sim_backend:=classic",
        " use_gazebo:=true",
        " robot_name:=rosmaster_x3",
        " prefix:=",
        f" plugin_ext:={PLUGIN_EXT}",
    ])

    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time.lower() in ("true", "1", "yes"),
            # Without value_type=str the launch system parses the URDF as YAML
            # and rejects it.
            "robot_description": ParameterValue(description, value_type=str),
        }],
    )]


def generate_launch_description():
    # This folder owns its world and RViz config, so resolve them next to this
    # file rather than through an installed package share.
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    default_world = os.path.join(here, "worlds", "classic_obstacles.world")
    default_rviz = os.path.join(here, "rviz", "classic.rviz")

    declare_world = DeclareLaunchArgument("world", default_value=default_world)
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Open the Gazebo Classic GUI")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="true")

    # libgazebo_ros_init provides /clock, libgazebo_ros_factory provides
    # /spawn_entity, libgazebo_ros_force_system provides the wrench services.
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            LaunchConfiguration("world"),
            "-s", f"libgazebo_ros_init{PLUGIN_EXT}",
            "-s", f"libgazebo_ros_factory{PLUGIN_EXT}",
            "-s", f"libgazebo_ros_force_system{PLUGIN_EXT}",
            "--verbose",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # -topic reads the URDF robot_state_publisher latches, so the spawned model
    # and the TF tree always describe the same robot.
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rosmaster_x3",
            # Wheel centres sit at base_footprint z=0 and the wheels have a
            # 0.0325 m radius, so this rests them exactly on the ground.
            "-z", "0.0325",
            # Default is 30 s; a cold conda dyld cache can take longer than that
            # to bring gzserver's /spawn_entity service up.
            "-timeout", "90.0",
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        declare_gui,
        declare_rviz,
        gzserver,
        gzclient,
        OpaqueFunction(function=_robot_description),
        # gzserver needs a moment to advertise /spawn_entity.
        TimerAction(period=7.0, actions=[spawn]),
        TimerAction(period=11.0, actions=[rviz]),
    ])
