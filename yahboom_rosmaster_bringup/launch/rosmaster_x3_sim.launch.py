#!/usr/bin/env python3
"""
Canonical simulation bringup entrypoint for Yahboom ROSMASTER X3.

Launches Gazebo Fortress simulation, robot description, controllers,
sensor bridges, wheel state odometry, and RViz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("yahboom_rosmaster_gazebo")
    fortress_launch = os.path.join(
        pkg_gazebo, "launch", "rosmaster_gazebo_fortress.launch.py")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="World file to load (empty.world, cafe.world, etc.)"
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo UI"
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Skip Gazebo UI client (true/false)"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 visualization"
    )
    motion_profile_arg = DeclareLaunchArgument(
        "motion_profile",
        default_value="stress",
        description="Wheel contact physics profile (ideal, stress)"
    )
    motion_bias_arg = DeclareLaunchArgument(
        "motion_bias",
        default_value="false",
        description="Enable motion bias / motor drift model"
    )
    sensor_profile_arg = DeclareLaunchArgument(
        "sensor_profile",
        default_value="physical",
        description=(
            "Sensor quality and timing profile (ideal, physical): physical "
            "delivers the point cloud with the physical X3's gaps and latency"
        )
    )
    sensor_seed_arg = DeclareLaunchArgument(
        "sensor_seed",
        default_value="-1",
        description="Seed for the sensor profiles' random draws (-1: random)"
    )
    cloud_strip_nan_arg = DeclareLaunchArgument(
        "cloud_strip_nan",
        default_value="true",
        description="Drop non-finite point cloud points (true/false)"
    )
    cloud_decimation_arg = DeclareLaunchArgument(
        "cloud_decimation",
        default_value="1",
        description="Keep every Nth row and column of the point cloud"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock"
    )
    ground_truth_frame_arg = DeclareLaunchArgument(
        "ground_truth_frame",
        default_value="auto",
        description=(
            "Ground-truth display frame: auto, odom, map, or another "
            "localization frame")
    )

    include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fortress_launch),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "motion_profile": LaunchConfiguration("motion_profile"),
            "motion_bias": LaunchConfiguration("motion_bias"),
            "sensor_profile": LaunchConfiguration("sensor_profile"),
            "sensor_seed": LaunchConfiguration("sensor_seed"),
            "cloud_strip_nan": LaunchConfiguration("cloud_strip_nan"),
            "cloud_decimation": LaunchConfiguration("cloud_decimation"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "ground_truth_frame": LaunchConfiguration("ground_truth_frame"),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        headless_arg,
        rviz_arg,
        motion_profile_arg,
        motion_bias_arg,
        sensor_profile_arg,
        sensor_seed_arg,
        cloud_strip_nan_arg,
        cloud_decimation_arg,
        use_sim_time_arg,
        ground_truth_frame_arg,
        include_sim,
    ])
