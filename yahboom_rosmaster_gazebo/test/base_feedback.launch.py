#!/usr/bin/env python3
"""Launch-test positive mecanum commands through wheel odometry and TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, "launch", "rosmaster_gazebo_fortress.launch.py")),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": os.path.join(package_share, "worlds", "empty.world"),
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="base_feedback_probe.py",
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_base_feedback_{os.getpid()}"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=15.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestBaseFeedback(unittest.TestCase):
    """Require the active base-feedback probe to pass."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=45)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
