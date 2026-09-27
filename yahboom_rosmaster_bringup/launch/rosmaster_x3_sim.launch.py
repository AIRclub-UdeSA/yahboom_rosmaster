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
        description="Launch Gazebo GUI client (true/false)"
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Skip Gazebo GUI client — server-only for autonomous/CI debugging"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz (true/false)"
    )
    motion_profile_arg = DeclareLaunchArgument(
        "motion_profile",
        default_value="stress",
        choices=["ideal", "stress"],
        description=(
            "Wheel-contact profile: stress is deterministic and uncalibrated; "
            "ideal preserves the zero-slip baseline"
        )
    )
    motion_bias_arg = DeclareLaunchArgument(
        "motion_bias",
        default_value="false",
        description="Enable motion bias / motor drift model"
    )
    sensor_profile_arg = DeclareLaunchArgument(
        "sensor_profile",
        default_value="physical",
        choices=["ideal", "physical"],
        description=(
            "Sensor quality and timing profile: physical delivers the point "
            "cloud with the physical X3's gaps and latency; ideal delivers "
            "every frame as soon as it is built"
        )
    )
    sensor_seed_arg = DeclareLaunchArgument(
        "sensor_seed",
        default_value="-1",
        description=(
            "Seed for the sensor profiles' random draws. -1 picks a random "
            "one per launch, which the camera adapter logs; tests pass a "
            "fixed one"
        )
    )
    cloud_strip_nan_arg = DeclareLaunchArgument(
        "cloud_strip_nan",
        default_value="true",
        choices=["true", "false"],
        description=(
            "Drop non-finite points from the point cloud, as the physical "
            "adapter does by default, leaving an unorganized dense cloud; "
            "false keeps the organized cloud"
        )
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
            "Diagnostic ground-truth display frame. 'auto' starts in odom and "
            "captures a fixed map alignment if map->odom appears; use 'odom' "
            "to disable the automatic SLAM switch"
        )
    )
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="0.0",
        description="Robot start x in the Gazebo world frame, in meters"
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0",
        description="Robot start y in the Gazebo world frame, in meters"
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0",
        description=(
            "Robot start heading in the Gazebo world frame, in radians "
            "(counterclockwise from +x). /odom still starts at zero"
        )
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
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
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
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        include_sim,
    ])
