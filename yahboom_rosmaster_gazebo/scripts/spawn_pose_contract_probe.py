#!/usr/bin/env python3
"""Validate that the robot spawns at the requested world-frame pose."""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


GROUND_TRUTH_TOPIC = "/ground_truth/odom"
ODOM_TOPIC = "/odom"


def yaw_from_quaternion(orientation):
    """Return the yaw angle of a quaternion, in radians."""
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def angle_difference(first, second):
    """Return the signed difference between two angles, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


class SpawnPoseContractProbe(Node):
    """Compare the first ground-truth and wheel-odometry poses to the request."""

    def __init__(self):
        super().__init__("spawn_pose_contract_probe")
        self.declare_parameter("timeout", 30.0)
        self.declare_parameter("expected_x", 0.0)
        self.declare_parameter("expected_y", 0.0)
        self.declare_parameter("expected_yaw", 0.0)
        self.declare_parameter("position_tolerance", 0.05)
        self.declare_parameter("yaw_tolerance", 0.05)
        self.timeout = float(self.get_parameter("timeout").value)
        self.expected_x = float(self.get_parameter("expected_x").value)
        self.expected_y = float(self.get_parameter("expected_y").value)
        self.expected_yaw = float(self.get_parameter("expected_yaw").value)
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)

        self.ground_truth = None
        self.odom = None

        best_effort_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscription_handles = [
            self.create_subscription(
                Odometry, GROUND_TRUTH_TOPIC, self.capture_ground_truth,
                best_effort_qos),
            self.create_subscription(
                Odometry, ODOM_TOPIC, self.capture_odom, best_effort_qos),
        ]
        self.get_logger().info(
            f"Expecting ({self.expected_x:.3f}, {self.expected_y:.3f}) "
            f"yaw {self.expected_yaw:.3f} on {GROUND_TRUTH_TOPIC}, and "
            f"{ODOM_TOPIC} starting at zero")

    def capture_ground_truth(self, message):
        """Keep the first ground-truth sample."""
        if self.ground_truth is None:
            self.ground_truth = message

    def capture_odom(self, message):
        """Keep the first wheel-odometry sample."""
        if self.odom is None:
            self.odom = message

    def complete(self):
        """Return whether both topics delivered a sample."""
        return self.ground_truth is not None and self.odom is not None

    def validate(self):
        """Return all observed spawn-pose contract violations."""
        errors = []
        if self.ground_truth is None:
            errors.append(f"{GROUND_TRUTH_TOPIC}: no messages received")
        if self.odom is None:
            errors.append(f"{ODOM_TOPIC}: no messages received")
        if errors:
            return errors

        position = self.ground_truth.pose.pose.position
        yaw = yaw_from_quaternion(self.ground_truth.pose.pose.orientation)
        position_error = math.hypot(
            position.x - self.expected_x, position.y - self.expected_y)
        yaw_error = abs(angle_difference(yaw, self.expected_yaw))
        if position_error > self.position_tolerance:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: position ({position.x:.3f}, "
                f"{position.y:.3f}) is {position_error:.3f} m from the "
                f"requested ({self.expected_x:.3f}, {self.expected_y:.3f})")
        if yaw_error > self.yaw_tolerance:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: yaw {yaw:.3f} rad is {yaw_error:.3f} "
                f"rad from the requested {self.expected_yaw:.3f}")

        odom_position = self.odom.pose.pose.position
        odom_yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        odom_distance = math.hypot(odom_position.x, odom_position.y)
        if odom_distance > self.position_tolerance:
            errors.append(
                f"{ODOM_TOPIC}: starts {odom_distance:.3f} m from zero; wheel "
                "odometry must not depend on the spawn pose")
        if abs(angle_difference(odom_yaw, 0.0)) > self.yaw_tolerance:
            errors.append(
                f"{ODOM_TOPIC}: starts at yaw {odom_yaw:.3f} rad; wheel "
                "odometry must not depend on the spawn pose")
        return errors


def main():
    rclpy.init()
    node = SpawnPoseContractProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)

        errors = node.validate()
        if errors:
            node.get_logger().error(
                "Spawn-pose contract FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info("Spawn-pose contract PASSED")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
