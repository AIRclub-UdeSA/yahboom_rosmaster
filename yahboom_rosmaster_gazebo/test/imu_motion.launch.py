#!/usr/bin/env python3
"""
Launch the empty-world simulator and require the IMU contract to pass.

The arguments choose the sensor profile, how many stationary raw samples grade
the noise, and whether to skip the motion phases (#43 step 8c).
"""

import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutdown_asserts import assert_clean_shutdown  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    sensor_profile = LaunchConfiguration("sensor_profile")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rosmaster_gazebo_fortress.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "render_sensors": "false",
            "sensor_profile": sensor_profile,
            "world": os.path.join(package_share, "worlds", "empty.world"),
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="imu_motion_probe.py",
        parameters=[{
            "timeout": 45.0,
            "stationary_samples": 20,
            "warmup_samples": 5,
            "nominal_rate": 10.0,
            "linear_command": 0.4,
            "linear_duration": 0.7,
            "yaw_command": 0.5,
            "yaw_duration": 2.0,
            "sensor_profile": ParameterValue(sensor_profile, value_type=str),
            "noise_samples": ParameterValue(
                LaunchConfiguration("noise_samples"), value_type=int),
            "skip_motion": ParameterValue(
                LaunchConfiguration("skip_motion"), value_type=bool),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sensor_profile", default_value="ideal", choices=["ideal", "physical"]),
        DeclareLaunchArgument("noise_samples", default_value="0"),
        DeclareLaunchArgument("skip_motion", default_value="false"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_imu_motion_{os.getpid()}"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=16.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestImuMotion(unittest.TestCase):
    """Require the IMU probe to finish successfully within the test budget."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=55)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        assert_clean_shutdown(proc_info)
