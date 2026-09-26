#!/usr/bin/env python3
"""Launch-test RGB-D response against a static known-geometry target."""

import os
import sys

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

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TEST_DIR)
sys.path.insert(0, os.path.join(os.path.dirname(TEST_DIR), "scripts"))

from real_robot_contract import RealRobotContract  # noqa: E402
from shutdown_asserts import assert_clean_shutdown  # noqa: E402


# The robot spawns with base_footprint at the world origin and the RGB-D
# camera renders from cam_1_color_frame, as the physical Astra registers depth
# to color. The 0.2 m deep target's front face lies at a known depth along the
# color optical axis, and the target is centred on cam_1_depth_frame so the
# 25.1 mm between the two apertures shows as parallax in the image.
CONTRACT = RealRobotContract.load()
COLOR_XYZ = CONTRACT.nominal("frames.mounts.cam_1_color_frame")["xyz"]
DEPTH_XYZ = CONTRACT.nominal("frames.mounts.cam_1_depth_frame")["xyz"]
BASE_LINK_Z = CONTRACT.nominal("frames.base_footprint_to_base_link_z_m")
TARGET_X = 0.9
TARGET_Y = DEPTH_XYZ[1]
TARGET_Z = BASE_LINK_Z + COLOR_XYZ[2]
# 0.1 is half the target's 0.2 m depth (the SDF box's x size), so the front
# face is at TARGET_X - 0.1. EXPECTED_DEPTH and TARGET_FACE_CENTER both use it.
EXPECTED_DEPTH = TARGET_X - 0.1 - COLOR_XYZ[0]
# The front face's centre on base_link.
TARGET_FACE_CENTER = (TARGET_X - 0.1, TARGET_Y, TARGET_Z - BASE_LINK_Z)

TARGET_SDF = """
<sdf version="1.7">
  <model name="depth_geometry_target">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.2 0.4 0.2</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 0.4 0.2</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 0 0 1</emissive>
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
        PythonLaunchDescriptionSource(os.path.join(
            package_share,
            "launch",
            "rosmaster_gazebo_fortress.launch.py",
        )),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": os.path.join(package_share, "worlds", "empty.world"),
        }.items(),
    )
    probe = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(package_share, "scripts", "depth_geometry_probe.py"),
            "--ros-args",
            "-p", "timeout:=45.0",
            "-p", "samples:=10",
            "-p", f"expected_depth:={EXPECTED_DEPTH}",
            "-p", "target_face_center:=[{}]".format(
                ", ".join(f"{value:.9f}" for value in TARGET_FACE_CENTER)),
        ],
        output="screen",
    )
    target = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", TARGET_SDF,
            "-name", "depth_geometry_target",
            "-x", str(TARGET_X),
            "-y", str(TARGET_Y),
            "-z", str(TARGET_Z),
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_depth_geometry_{os.getpid()}"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", str(20 + os.getpid() % 200)),
        simulator,
        # Collect several target-free frames before inserting the red box.
        TimerAction(period=14.0, actions=[probe]),
        TimerAction(period=22.0, actions=[target]),
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe, "target": target}


class TestDepthGeometry(unittest.TestCase):
    """Require both target insertion and the geometry probe to succeed."""

    def test_target_spawned(self, proc_info, target):
        proc_info.assertWaitForStartup(target, timeout=30)
        proc_info.assertWaitForShutdown(target, timeout=15)
        launch_testing.asserts.assertExitCodes(proc_info, process=target)

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=55)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Require every launched process to exit cleanly."""

    def test_all_processes_exit_cleanly(self, proc_info):
        assert_clean_shutdown(proc_info)
