#!/usr/bin/env python3
"""Check that /imu/data_raw carries what the physical driver publishes there."""

from pathlib import Path
import sys
import unittest

import rclpy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Imu

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from imu_raw_relay import ImuRawRelay  # noqa: E402
from real_robot_contract import RealRobotContract  # noqa: E402


def gazebo_style_message():
    """Return an IMU message as the bridge delivers Gazebo's: oriented, with covariance."""
    message = Imu()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 300_000_000
    message.header.frame_id = "imu_link"
    message.orientation.x, message.orientation.y = -0.5, -0.5
    message.orientation.z, message.orientation.w = 0.5, 0.5
    message.orientation_covariance = [0.1] * 9
    message.angular_velocity.x, message.angular_velocity.y = 0.01, -0.02
    message.angular_velocity.z = 0.03
    message.angular_velocity_covariance = [0.2] * 9
    message.linear_acceleration.x, message.linear_acceleration.y = 0.1, -0.2
    message.linear_acceleration.z = -9.8
    message.linear_acceleration_covariance = [0.3] * 9
    return message


class TestImuRawRelay(unittest.TestCase):
    """The relay copies the three fields the driver fills, and nothing else."""

    def setUp(self):
        rclpy.init()
        self.relay = ImuRawRelay()
        self.published = []
        self.relay.publisher.publish = self.published.append

    def tearDown(self):
        self.relay.destroy_node()
        rclpy.shutdown()

    def test_the_header_and_both_measurements_are_copied(self):
        source = gazebo_style_message()
        self.relay.republish(source)
        raw = self.published[0]
        self.assertEqual(raw.header, source.header)
        self.assertEqual(raw.angular_velocity, source.angular_velocity)
        self.assertEqual(raw.linear_acceleration, source.linear_acceleration)

    def test_the_orientation_is_the_identity_the_driver_leaves(self):
        self.relay.republish(gazebo_style_message())
        orientation = self.published[0].orientation
        self.assertEqual(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            (0.0, 0.0, 0.0, 1.0))

    def test_all_three_covariances_are_zero(self):
        self.relay.republish(gazebo_style_message())
        raw = self.published[0]
        for covariance in (
                raw.orientation_covariance, raw.angular_velocity_covariance,
                raw.linear_acceleration_covariance):
            self.assertEqual(list(covariance), [0.0] * 9)

    def test_the_content_is_the_ledgers(self):
        ledger = RealRobotContract.load()
        self.relay.republish(gazebo_style_message())
        raw = self.published[0]
        expected = ledger.physical("topics./imu/data_raw.orientation")
        self.assertEqual(
            (raw.orientation.x, raw.orientation.y, raw.orientation.z, raw.orientation.w),
            (expected["x"], expected["y"], expected["z"], expected["w"]))
        self.assertEqual(ledger.physical("topics./imu/data_raw.covariances"), "all_zero")

    def test_it_publishes_reliable_with_the_drivers_depth(self):
        profile = self.relay.publisher.qos_profile
        self.assertEqual(profile.reliability, QoSReliabilityPolicy.RELIABLE)
        self.assertEqual(profile.depth, 100)

    def test_it_listens_to_its_input_only(self):
        # No /clock subscription and no timer: the relay only reacts to a message.
        self.assertEqual(
            [s.topic_name for s in self.relay.subscriptions], ["/internal/imu/data_raw"])
        self.assertEqual(list(self.relay.timers), [])
        self.assertFalse(self.relay.get_parameter("use_sim_time").value)


if __name__ == "__main__":
    unittest.main()
