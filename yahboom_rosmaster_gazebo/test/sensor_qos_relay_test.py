#!/usr/bin/env python3
"""Check the scan relay's hold-back and scan_time, without a simulator."""

from pathlib import Path
import sys
import unittest

import rclpy
from sensor_msgs.msg import Image, LaserScan

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from sensor_qos_relay import HoldBack, SensorQosRelay  # noqa: E402

SCAN_PERIOD_S = 0.14


class TestHoldBack(unittest.TestCase):
    """The three edges of publishing each message when the next one arrives."""

    def test_a_count_of_zero_releases_every_message_at_once(self):
        hold = HoldBack(0)
        self.assertEqual([hold.push(index) for index in range(3)], [0, 1, 2])

    def test_a_count_of_one_releases_each_message_when_the_next_arrives(self):
        hold = HoldBack(1)
        self.assertEqual([hold.push(index) for index in range(4)], [None, 0, 1, 2])

    def test_the_first_message_waits_for_the_second(self):
        hold = HoldBack(1)
        self.assertIsNone(hold.push("first"))
        self.assertEqual(hold.push("second"), "first")

    def test_the_last_message_is_never_released(self):
        hold = HoldBack(1)
        released = [hold.push(index) for index in range(5)]
        self.assertNotIn(4, released)
        self.assertEqual(list(hold.held), [4])

    def test_a_message_that_never_arrives_makes_its_predecessor_wait_two_periods(self):
        # Arrivals at 0, 1, 3 and 4 periods: message 1 waits for message 3.
        arrivals = [0, 1, 3, 4]
        hold = HoldBack(1)
        waits = {}
        for index, arrival in zip((0, 1, 3, 4), arrivals):
            released = hold.push(index)
            if released is not None:
                waits[released] = arrival - arrivals[(0, 1, 3, 4).index(released)]
        self.assertEqual(waits, {0: 1, 1: 2, 3: 1})

    def test_order_is_preserved(self):
        hold = HoldBack(2)
        released = [hold.push(index) for index in range(6)]
        self.assertEqual([r for r in released if r is not None], [0, 1, 2, 3])

    def test_a_negative_count_is_rejected(self):
        with self.assertRaises(ValueError):
            HoldBack(-1)


class TestRelayNode(unittest.TestCase):
    """The node publishes held scans with their own stamps and the robot's scan_time."""

    def tearDown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def make_relay(self, msg_type="LaserScan", hold=1, scan_time=0.1343):
        """Start a relay on a fresh context, with its publisher recording."""
        if rclpy.ok():
            rclpy.shutdown()
        parameters = [
            "-p", f"msg_type:={msg_type}",
            "-p", "input_topic:=/internal/scan",
            "-p", "output_topic:=/scan",
            "-p", f"hold_scans:={hold}",
        ]
        if scan_time is not None:
            parameters += ["-p", f"scan_time:={scan_time}"]
        rclpy.init(args=["--ros-args", *parameters])
        relay = SensorQosRelay()
        relay.published = []
        relay.publisher.publish = relay.published.append
        return relay

    @staticmethod
    def scan(index):
        message = LaserScan()
        message.header.stamp.sec = 10
        message.header.stamp.nanosec = int(index * SCAN_PERIOD_S * 1e9)
        message.scan_time = 0.0
        return message

    def test_each_scan_leaves_with_the_next_and_keeps_its_stamp(self):
        relay = self.make_relay()
        for index in range(4):
            relay.republish(self.scan(index))
        self.assertEqual(len(relay.published), 3)
        self.assertEqual(
            [m.header.stamp.nanosec for m in relay.published],
            [int(i * SCAN_PERIOD_S * 1e9) for i in range(3)])
        relay.destroy_node()

    def test_the_scan_time_is_the_profile_value_whatever_the_bridge_sent(self):
        relay = self.make_relay(hold=0)
        relay.republish(self.scan(0))
        self.assertAlmostEqual(relay.published[0].scan_time, 0.1343)
        relay.destroy_node()

    def test_without_a_scan_time_the_message_keeps_its_own(self):
        relay = self.make_relay(hold=0, scan_time=None)
        message = self.scan(0)
        message.scan_time = 0.5
        relay.republish(message)
        self.assertEqual(relay.published[0].scan_time, 0.5)
        relay.destroy_node()

    def test_scan_time_is_refused_for_another_message_type(self):
        with self.assertRaisesRegex(ValueError, "scan_time only applies"):
            self.make_relay(msg_type="Image", hold=0, scan_time=0.1)

    def test_the_relay_listens_to_its_input_only(self):
        # No /clock subscription and no timer: an rclpy node on sim time wakes on
        # every tick of the 1 kHz clock, and the hold needs no clock at all.
        relay = self.make_relay()
        self.assertEqual([s.topic_name for s in relay.subscriptions], ["/internal/scan"])
        self.assertEqual(list(relay.timers), [])
        self.assertFalse(relay.get_parameter("use_sim_time").value)
        relay.destroy_node()

    def test_another_relay_is_unchanged_by_the_defaults(self):
        relay = self.make_relay(msg_type="Image", hold=0, scan_time=None)
        image = Image()
        relay.republish(image)
        self.assertEqual(relay.published, [image])
        relay.destroy_node()


if __name__ == "__main__":
    unittest.main()
