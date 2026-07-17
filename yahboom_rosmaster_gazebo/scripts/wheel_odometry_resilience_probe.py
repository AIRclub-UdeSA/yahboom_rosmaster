#!/usr/bin/env python3
"""Inject wheel states and validate odometry rewind/discontinuity handling."""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


WHEEL_NAMES = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
)


def stamp_key(stamp):
    """Return an exact timestamp key."""
    return (stamp.sec, stamp.nanosec)


class WheelOdometryResilienceProbe(Node):
    """Publish deterministic encoder sequences and inspect odometry plus TF."""

    def __init__(self):
        super().__init__("wheel_odometry_resilience_probe")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.odom_messages = {}
        self.transforms = {}
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.capture_odometry, 10)
        self.tf_subscription = self.create_subscription(
            TFMessage, "/tf", self.capture_tf, 20)

    def capture_odometry(self, message):
        self.odom_messages[stamp_key(message.header.stamp)] = message

    def capture_tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id == "odom" and
                    transform.child_frame_id == "base_footprint"):
                self.transforms[stamp_key(transform.header.stamp)] = transform

    @staticmethod
    def joint_state(seconds, positions):
        message = JointState()
        message.header.stamp.sec = int(seconds)
        message.header.stamp.nanosec = int(round((seconds - int(seconds)) * 1e9))
        message.header.frame_id = "base_link"
        message.name = list(WHEEL_NAMES)
        message.position = [float(value) for value in positions]
        message.velocity = [0.0] * 4
        return message

    def publish_and_wait(self, seconds, positions, timeout=2.0):
        message = self.joint_state(seconds, positions)
        key = stamp_key(message.header.stamp)
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(message)
                next_publish = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.02)
            if key in self.odom_messages and key in self.transforms:
                return self.odom_messages[key], self.transforms[key]
        return None, None

    @staticmethod
    def validate_pair(label, odometry, transform, expected_x, expected_twist, errors):
        if odometry is None or transform is None:
            errors.append(f"{label}: missing odometry or matching TF")
            return
        if not math.isclose(odometry.pose.pose.position.x, expected_x, abs_tol=1e-9):
            errors.append(
                f"{label}: odom x={odometry.pose.pose.position.x:.9f}, "
                f"expected {expected_x:.9f}")
        if not math.isclose(
                odometry.twist.twist.linear.x, expected_twist, abs_tol=1e-9):
            errors.append(
                f"{label}: odom vx={odometry.twist.twist.linear.x:.9f}, "
                f"expected {expected_twist:.9f}")
        if stamp_key(odometry.header.stamp) != stamp_key(transform.header.stamp):
            errors.append(f"{label}: odometry and TF stamps differ")
        if not math.isclose(
                transform.transform.translation.x, expected_x, abs_tol=1e-9):
            errors.append(f"{label}: odometry and TF poses differ")


def main():
    rclpy.init()
    node = WheelOdometryResilienceProbe()
    errors = []
    try:
        ready_deadline = time.monotonic() + 5.0
        while (rclpy.ok() and time.monotonic() < ready_deadline and
               node.publisher.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.publisher.get_subscription_count() == 0:
            node.get_logger().error("Wheel odometry resilience FAILED: no subscriber")
            return 1

        cases = (
            ("initial", 10.0, (0.0,) * 4, 0.0, 0.0),
            ("integrated", 10.1, (1.0,) * 4, 0.0325, 0.325),
            # A time rewind must reset integrated pose and rebase encoders.
            ("rewind", 1.0, (0.2,) * 4, 0.0, 0.0),
            ("after rewind", 1.1, (1.2,) * 4, 0.0325, 0.325),
            # A large position reset must not create a false odometry jump.
            ("discontinuity", 1.2, (100.0,) * 4, 0.0325, 0.0),
            ("after discontinuity", 1.3, (101.0,) * 4, 0.065, 0.325),
        )
        for label, stamp, positions, expected_x, expected_twist in cases:
            odometry, transform = node.publish_and_wait(stamp, positions)
            node.validate_pair(
                label, odometry, transform, expected_x, expected_twist, errors)

        if errors:
            node.get_logger().error(
                "Wheel odometry resilience FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info(
            "Wheel odometry resilience PASSED: normal integration, clock rewind, "
            "joint discontinuity, and exact odom/TF pairs")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
