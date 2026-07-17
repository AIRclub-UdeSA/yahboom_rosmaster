#!/usr/bin/env python3
"""Launch-test wheel odometry reset and discontinuity resilience."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    odometry = ExecuteProcess(
        cmd=["python3", os.path.join(
            package_share, "scripts", "wheel_state_odometry.py")],
        output="screen",
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="wheel_odometry_resilience_probe.py",
        output="screen",
    )
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        odometry,
        TimerAction(period=1.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestWheelOdometryResilience(unittest.TestCase):
    """Require the deterministic resilience sequence to pass."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=10)
        proc_info.assertWaitForShutdown(probe, timeout=20)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)
