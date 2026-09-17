#!/usr/bin/env python3
"""Launch-test fixed ground-truth display alignment without Gazebo."""

import os
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest


@pytest.mark.launch_test
def generate_test_description():
    helper = Node(
        package="yahboom_rosmaster_gazebo",
        executable="ground_truth_tf.py",
        parameters=[{
            "frame_id": "auto",
            "alignment_timeout": 1.0,
        }],
        output="screen",
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="ground_truth_display_probe.py",
        output="screen",
    )
    return LaunchDescription([
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        helper,
        TimerAction(period=0.5, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestGroundTruthDisplay(unittest.TestCase):
    """Require the synthetic alignment probe to pass."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=10)
        proc_info.assertWaitForShutdown(probe, timeout=20)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
