#!/usr/bin/env python3
"""Launch a practice world headless and require the spawn smoke contract to pass."""

import os

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
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    world = LaunchConfiguration("world")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rosmaster_gazebo_fortress.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": PathJoinSubstitution([package_share, "worlds", world]),
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="world_smoke_probe.py",
        parameters=[{"timeout": 25.0, "settle_window": 3.0}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.world"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_world_smoke_{os.getpid()}"),
        # Keep sequential world tests isolated even when their PIDs differ by a
        # round multiple of 100 (a pattern observed under CTest).
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=15.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestWorldSmoke(unittest.TestCase):
    """Require the spawn smoke probe to finish successfully within the launch-test budget."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=35)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Reject signal escalation or orphan-prone simulator process exits."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
