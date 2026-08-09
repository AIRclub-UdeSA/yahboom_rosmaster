#!/usr/bin/env python3
"""
SLAM (and optionally Nav2) on top of the Gazebo Classic backend.

Start the simulation first, then this. The lidar feeding it is Gazebo Classic's
CPU raycast sensor, the only lidar that works on macOS.

  ./run slam      # mapping only
  ./run nav2      # mapping plus navigation
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory("yahboom_rosmaster_navigation")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    default_params = os.path.join(
        pkg_nav, "config", "rosmaster_x3_nav2_default_params.yaml")

    declare_params = DeclareLaunchArgument("params_file", default_value=default_params)
    declare_nav2 = DeclareLaunchArgument(
        "nav2", default_value="false",
        description="Also bring up Nav2 (needs SLAM to have published a map)")

    params_file = LaunchConfiguration("params_file")

    # mode: mapping, base_frame: base_footprint, scan_topic: /scan are already
    # set in the shared params file, so the same config serves both backends.
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[params_file, {"use_sim_time": True}],
    )

    # Nav2 waits for the map->odom transform SLAM provides, so start it later.
    nav2 = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": params_file,
                "autostart": "true",
            }.items(),
        )],
        condition=IfCondition(LaunchConfiguration("nav2")),
    )

    return LaunchDescription([declare_params, declare_nav2, slam, nav2])
