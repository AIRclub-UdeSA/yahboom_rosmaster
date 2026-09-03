#!/usr/bin/env python3
"""Launch the standalone simulator and require its sensor contract to pass."""

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
from launch_ros.parameter_descriptions import ParameterValue
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    world = LaunchConfiguration("world")
    samples = LaunchConfiguration("samples")
    performance_checks = LaunchConfiguration("performance_checks")
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
        executable="sensor_contract_probe.py",
        parameters=[{
            "timeout": 45.0,
            "samples": ParameterValue(samples, value_type=int),
            "performance_checks": ParameterValue(
                performance_checks, value_type=bool),
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.world"),
        DeclareLaunchArgument("samples", default_value="10"),
        DeclareLaunchArgument("performance_checks", default_value="true"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_sensor_contract_{os.getpid()}"),
        # Keep sequential world tests isolated even when their PIDs differ by a
        # round multiple of 100 (a pattern observed under CTest).
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=15.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestSensorContract(unittest.TestCase):
    """Require the probe to finish successfully within the launch-test budget."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=55)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Reject signal escalation or orphan-prone simulator process exits."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
