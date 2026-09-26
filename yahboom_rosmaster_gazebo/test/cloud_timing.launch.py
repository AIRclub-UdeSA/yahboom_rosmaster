#!/usr/bin/env python3
"""Launch the simulator under a sensor profile and grade its point cloud timing."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutdown_asserts import assert_clean_shutdown  # noqa: E402
from sim_timing import PERFORMANCE_PROBE_START_DELAY  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    profile = LaunchConfiguration("sensor_profile")
    duration = LaunchConfiguration("duration")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rosmaster_gazebo_fortress.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": PathJoinSubstitution([package_share, "worlds", "empty.world"]),
            "sensor_profile": profile,
            "cloud_decimation": LaunchConfiguration("cloud_decimation"),
            # A fixed seed, so a failure reproduces its gap sequence.
            "sensor_seed": "20260926",
            "motion_bias": "false",
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="cloud_timing_probe.py",
        parameters=[{
            "use_sim_time": True,
            "profile": profile,
            "duration_s": ParameterValue(duration, value_type=float),
            "timeout": 200.0,
            "report_path": LaunchConfiguration("report_path"),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("sensor_profile", default_value="physical"),
        DeclareLaunchArgument(
            "duration", default_value="60",
            description="Sim seconds of camera frames to grade"),
        DeclareLaunchArgument("report_path", default_value=""),
        # The ideal run delivers 30 clouds a second. Whole clouds of 600 kB are
        # sometimes lost between processes, so it keeps every 4th row and column
        # to measure the profile and not the transport.
        DeclareLaunchArgument("cloud_decimation", default_value="1"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_cloud_timing_{os.getpid()}"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(30 + os.getpid() % 180)),
        simulator,
        TimerAction(period=PERFORMANCE_PROBE_START_DELAY, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestCloudTiming(unittest.TestCase):
    """Require the cloud's gaps and latency to match the profile on sim time."""

    def test_timing_matches_the_profile(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=40)
        proc_info.assertWaitForShutdown(probe, timeout=220)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        assert_clean_shutdown(proc_info)
