#!/usr/bin/env python3
"""Focused unit tests for the practice-world smoke probe."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

import world_smoke_probe  # noqa: E402
from world_smoke_probe import WorldSmokeProbe, stamp_seconds  # noqa: E402


def odometry(stamp, x=0.0):
    """Build a level ground-truth sample at a requested time and x position."""
    message = Odometry()
    message.header.stamp.sec = int(stamp)
    message.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    message.pose.pose.position.x = x
    message.pose.pose.orientation.w = 1.0
    return message


class TestWorldSmokeProbe(unittest.TestCase):
    """Exercise validation boundaries without launching Gazebo."""

    def setUp(self):
        self.probe = WorldSmokeProbe.__new__(WorldSmokeProbe)
        self.probe.settle_window = 3.0
        self.probe.command_duration = 2.0
        self.probe.meaningful_motion = 0.08
        self.probe.ground_truth = []
        self.probe.latest_ground_truth = None
        self.probe.essential_seen = {
            "/odom": True,
            "/tf": True,
            "/joint_states": True,
            "/scan": True,
            "/imu/data": True,
        }

    def test_capture_includes_first_sample_past_settle_boundary(self):
        for stamp in (0.0, 2.98, 3.02, 3.04):
            self.probe.capture_ground_truth(odometry(stamp))

        self.assertEqual(len(self.probe.ground_truth), 3)
        elapsed = (
            stamp_seconds(self.probe.ground_truth[-1].header.stamp)
            - stamp_seconds(self.probe.ground_truth[0].header.stamp)
        )
        self.assertAlmostEqual(elapsed, 3.02)

    def test_validate_rejects_incomplete_settle_window(self):
        self.probe.ground_truth = [odometry(0.0)]

        errors = self.probe.validate()

        self.assertTrue(any("only observed" in error for error in errors))

    def test_validate_rejects_displaced_spawn(self):
        self.probe.ground_truth = [odometry(0.0, x=2.0), odometry(3.0, x=2.0)]

        errors = self.probe.validate()

        self.assertTrue(any("expected (0, 0) spawn point" in error for error in errors))

    def test_motion_requires_full_simulation_time_window(self):
        start = odometry(0.0)
        end = odometry(0.5, x=0.20)

        errors = self.probe.validate_motion(start, end, command_elapsed=0.5)

        self.assertTrue(any("simulation advanced only" in error for error in errors))

    def test_publish_command_uses_simulation_time_window(self):
        class RecordingPublisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        self.probe.latest_ground_truth = odometry(10.0)
        self.probe.command_publisher = RecordingPublisher()
        next_stamp = 10.0

        def advance_simulation(_node, timeout_sec):
            nonlocal next_stamp
            self.assertEqual(timeout_sec, 0.05)
            next_stamp += 0.25
            self.probe.latest_ground_truth = odometry(next_stamp)

        command = Twist()
        command.linear.x = 0.15
        with (
            patch.object(world_smoke_probe.rclpy, "ok", return_value=True),
            patch.object(world_smoke_probe.rclpy, "spin_once", advance_simulation),
            patch.object(world_smoke_probe.time, "monotonic", return_value=0.0),
        ):
            elapsed = self.probe.publish_command(
                command, duration=2.0, wall_deadline=10.0)

        self.assertAlmostEqual(elapsed, 2.0)
        self.assertEqual(len(self.probe.command_publisher.messages), 13)
        commanded = self.probe.command_publisher.messages[:8]
        stopped = self.probe.command_publisher.messages[8:]
        self.assertTrue(all(message.linear.x == 0.15 for message in commanded))
        self.assertTrue(all(message.linear.x == 0.0 for message in stopped))

    def test_motion_requires_meaningful_displacement(self):
        start = odometry(0.0)
        end = odometry(2.0, x=0.02)

        errors = self.probe.validate_motion(start, end, command_elapsed=2.0)

        self.assertTrue(any("wheels may be spinning" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
