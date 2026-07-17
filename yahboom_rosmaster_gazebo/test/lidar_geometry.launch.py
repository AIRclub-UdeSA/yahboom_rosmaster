#!/usr/bin/env python3
"""Launch an isolated empty-world LiDAR known-geometry acceptance test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest


FRONT_TARGET_SDF = """
<sdf version="1.8">
  <model name="lidar_front_target">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.2 0.6 1.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 0.6 1.0</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

LEFT_TARGET_SDF = """
<sdf version="1.8">
  <model name="lidar_left_target">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.2 0.3 1.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 0.3 1.0</size></box></geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "rosmaster_gazebo_fortress.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": os.path.join(package_share, "worlds", "empty.world"),
        }.items(),
    )

    front_target = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty_world",
            "-string", FRONT_TARGET_SDF,
            "-name", "lidar_front_target",
            "-x", "2.0", "-y", "0.0", "-z", "0.5",
        ],
        output="screen",
    )
    left_target = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty_world",
            "-string", LEFT_TARGET_SDF,
            "-name", "lidar_left_target",
            "-x", "2.4", "-y", "1.2", "-z", "0.5",
        ],
        output="screen",
    )
    probe = ExecuteProcess(
        cmd=[
            "python3", os.path.join(package_share, "scripts", "lidar_geometry_probe.py"),
            "--ros-args", "-p", "timeout:=30.0", "-p", "samples:=12",
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_lidar_geometry_{os.getpid()}"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        simulator,
        TimerAction(period=6.0, actions=[front_target, left_target]),
        TimerAction(period=15.0, actions=[probe]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestLidarGeometry(unittest.TestCase):
    """Require the known-geometry probe to finish successfully."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=35)
        proc_info.assertWaitForShutdown(probe, timeout=40)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
