#!/usr/bin/env python3
"""Launch-test profile-specific odometry divergence during a strafe."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    motion_profile = LaunchConfiguration("motion_profile")
    min_error = LaunchConfiguration("min_error")
    max_error = LaunchConfiguration("max_error")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "rosmaster_gazebo_fortress.launch.py",
            )
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": os.path.join(package_share, "worlds", "empty.world"),
            "motion_profile": motion_profile,
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="motion_profile_divergence_probe.py",
        parameters=[{
            "profile": ParameterValue(motion_profile, value_type=str),
            "timeout": 40.0,
            "command_speed": 0.2,
            "command_duration": 3.0,
            "initial_settle_duration": 1.0,
            "final_settle_duration": 1.0,
            "meaningful_motion": 0.3,
            "min_translation_error": ParameterValue(
                min_error, value_type=float),
            "max_translation_error": ParameterValue(
                max_error, value_type=float),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("motion_profile", default_value="stress"),
        DeclareLaunchArgument("min_error", default_value="0.003"),
        DeclareLaunchArgument("max_error", default_value="0.030"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_profile_divergence_{os.getpid()}"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=15.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestMotionProfileDivergence(unittest.TestCase):
    """Require the profile-specific motion error bound to pass."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=50)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
