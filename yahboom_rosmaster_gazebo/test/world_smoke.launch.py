#!/usr/bin/env python3
"""Launch a practice world headless and require its smoke contract to pass."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutdown_asserts import assert_clean_shutdown  # noqa: E402


GROUND_TRUTH_READY_TIMEOUT = 20.0


def _start_probe_after_ground_truth(
        event, context, probe, readiness_timeout):
    """Start the probe after the simulator publishes robot pose."""
    readiness_timeout.cancel()
    # A timeout-triggered launch shutdown can make the readiness process exit
    # while this handler is still registered. Never start the probe during
    # teardown or emit a redundant Shutdown event.
    if context.is_shutdown:
        return []
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(
            reason=(
                "ground-truth readiness process exited with code "
                f"{event.returncode} before the smoke probe could start"
            ),
        ))]
    return [
        LogInfo(
            msg="Ground truth is publishing; starting the world smoke probe"),
        probe,
    ]


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    world = LaunchConfiguration("world")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share, "launch", "rosmaster_gazebo_fortress.launch.py")
        ),
        launch_arguments={
            "headless": "true",
            "rviz": "false",
            "use_sim_time": "true",
            "world": PathJoinSubstitution([package_share, "worlds", world]),
            # The probe commands a short forward move to confirm the world
            # doesn't block translation (world_smoke_probe.py checks this
            # against a fixed meaningful_motion distance), so the /cmd_vel
            # motion bias, resampled at random per launch, must be off.
            "motion_bias": "false",
        }.items(),
    )
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="world_smoke_probe.py",
        parameters=[{
            "timeout": 32.0,
            "settle_window": 3.0,
            "command_speed": 0.15,
            "command_duration": 2.0,
            "meaningful_motion": 0.08,
        }],
        output="screen",
    )
    # Start listening before the simulator so the probe is launched on the
    # first bridged robot-pose sample instead of after an assumed wall-clock
    # startup duration. Ground truth is emitted only after Gazebo has spawned
    # the robot and the bridge is active; the probe itself then owns the richer
    # settle/interface checks.
    ground_truth_ready = ExecuteProcess(
        cmd=[
            "ros2", "topic", "echo",
            "--once",
            "--no-daemon",
            "--qos-reliability", "best_effort",
            "--field", "header.stamp",
            "/ground_truth/odom", "nav_msgs/msg/Odometry",
        ],
        name="wait_for_ground_truth",
        output="screen",
    )
    # Bound the readiness gate before the active test's 30-second startup wait.
    # Together with the probe's 32-second budget, this leaves 18 seconds inside
    # each CTest target's 70-second limit for launch and simulator teardown.
    readiness_timeout = TimerAction(
        period=GROUND_TRUTH_READY_TIMEOUT,
        actions=[
            LogInfo(msg=(
                "Timed out waiting for /ground_truth/odom; "
                "shutting down the world smoke test")),
            EmitEvent(event=Shutdown(
                reason=(
                    "ground truth did not become ready within "
                    f"{GROUND_TRUTH_READY_TIMEOUT:.0f} seconds"))),
        ],
    )
    start_probe_when_ready = RegisterEventHandler(OnProcessExit(
        target_action=ground_truth_ready,
        on_exit=lambda event, context: _start_probe_after_ground_truth(
            event, context, probe, readiness_timeout),
    ))

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.world"),
        SetEnvironmentVariable(
            "IGN_PARTITION", f"yahboom_world_smoke_{os.getpid()}"),
        # Keep sequential world tests isolated even when their PIDs differ by a
        # round multiple of 100 (a pattern observed under CTest).
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        start_probe_when_ready,
        readiness_timeout,
        ground_truth_ready,
        simulator,
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestWorldSmoke(unittest.TestCase):
    """Require the probe to finish within the launch-test budget."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=30)
        proc_info.assertWaitForShutdown(probe, timeout=42)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Reject signal escalation or orphan-prone simulator process exits."""

    def test_all_processes_exit_cleanly(self, proc_info):
        assert_clean_shutdown(proc_info)
