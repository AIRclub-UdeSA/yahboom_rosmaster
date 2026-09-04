#!/usr/bin/env python3
"""Exercise ground-truth display alignment with synthetic TF changes."""

import math
import sys
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from ground_truth_alignment import RigidTransform, compose, inverse, yaw_quaternion


def quaternion_distance(left, right):
    """Return sign-insensitive Euclidean quaternion distance."""
    direct = math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))
    negated = math.sqrt(sum((a + b) ** 2 for a, b in zip(left, right)))
    return min(direct, negated)


def close_transform(actual, expected, tolerance=1e-5):
    """Return whether translation and rotation agree within tolerance."""
    return (
        math.dist(actual.translation, expected.translation) <= tolerance
        and quaternion_distance(actual.rotation, expected.rotation) <= tolerance
    )


class GroundTruthDisplayProbe(Node):
    """Publish a nonidentity spawn and a changing synthetic map correction."""

    def __init__(self):
        super().__init__("ground_truth_display_probe")
        self.truth_publisher = self.create_publisher(
            Odometry, "/ground_truth/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, "/tf", self.capture_tf, qos)

        self.world_from_base_initial = RigidTransform(
            (7.0, 4.0, 0.2), yaw_quaternion(-0.6))
        self.odom_from_base = RigidTransform(
            (1.0, -2.0, 0.1), yaw_quaternion(0.25))
        self.world_from_base_at_map_start = compose(
            self.world_from_base_initial,
            RigidTransform((0.4, 0.1, 0.0), yaw_quaternion(0.12)),
        )
        self.odom_from_base_at_map_start = compose(
            self.odom_from_base,
            RigidTransform((0.45, 0.08, 0.0), yaw_quaternion(0.14)),
        )
        self.first_map_from_odom = RigidTransform(
            (3.0, 1.0, 0.0), yaw_quaternion(0.4))
        self.changed_map_from_odom = RigidTransform(
            (30.0, -10.0, 0.0), yaw_quaternion(-1.0))
        self.world_from_base_later = compose(
            self.world_from_base_at_map_start,
            RigidTransform((0.6, -0.2, 0.0), yaw_quaternion(0.15)),
        )
        self.fixed_map_from_world = compose(
            compose(self.first_map_from_odom, self.odom_from_base_at_map_start),
            inverse(self.world_from_base_at_map_start),
        )

        self.phase = "odom"
        self.phase_started = time.monotonic()
        self.matches = 0
        self.mismatches = 0
        self.done = False
        self.error = None
        self.create_timer(0.02, self.publish_inputs)

    @staticmethod
    def to_message(parent, child, value, stamp):
        """Convert a transform value into a stamped ROS transform."""
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = value.translation[0]
        message.transform.translation.y = value.translation[1]
        message.transform.translation.z = value.translation[2]
        message.transform.rotation.x = value.rotation[0]
        message.transform.rotation.y = value.rotation[1]
        message.transform.rotation.z = value.rotation[2]
        message.transform.rotation.w = value.rotation[3]
        return message

    @staticmethod
    def from_message(message):
        """Convert a ROS transform into a transform value."""
        value = message.transform
        return RigidTransform(
            (value.translation.x, value.translation.y, value.translation.z),
            (value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w),
        )

    def publish_inputs(self):
        """Publish synchronized truth, odometry TF, and phased SLAM TF."""
        if self.done:
            return
        if time.monotonic() - self.phase_started > 6.0:
            self.fail(f"timed out in {self.phase!r} phase")
            return

        stamp = self.get_clock().now().to_msg()
        odom_from_base = (
            self.odom_from_base
            if self.phase == "odom"
            else self.odom_from_base_at_map_start
        )
        transforms = [self.to_message(
            "odom", "base_footprint", odom_from_base, stamp)]
        if self.phase == "map_initial":
            transforms.append(self.to_message(
                "map", "odom", self.first_map_from_odom, stamp))
        elif self.phase == "map_changed":
            transforms.append(self.to_message(
                "map", "odom", self.changed_map_from_odom, stamp))
        self.tf_broadcaster.sendTransform(transforms)

        if self.phase == "odom":
            world_from_base = self.world_from_base_initial
        elif self.phase == "map_initial":
            world_from_base = self.world_from_base_at_map_start
        else:
            world_from_base = self.world_from_base_later
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = "world"
        truth.child_frame_id = "base_footprint"
        truth.pose.pose.position.x = world_from_base.translation[0]
        truth.pose.pose.position.y = world_from_base.translation[1]
        truth.pose.pose.position.z = world_from_base.translation[2]
        truth.pose.pose.orientation.x = world_from_base.rotation[0]
        truth.pose.pose.orientation.y = world_from_base.rotation[1]
        truth.pose.pose.orientation.z = world_from_base.rotation[2]
        truth.pose.pose.orientation.w = world_from_base.rotation[3]
        self.truth_publisher.publish(truth)

    def capture_tf(self, message):
        """Validate the diagnostic child in each alignment phase."""
        for transform in message.transforms:
            if transform.child_frame_id != "ground_truth_base":
                continue
            if self.phase == "odom" and transform.header.frame_id == "odom":
                self.record(self.from_message(transform), self.odom_from_base)
            elif (
                    self.phase == "map_initial"
                    and transform.header.frame_id == "map"):
                self.record(
                    self.from_message(transform),
                    compose(
                        self.first_map_from_odom,
                        self.odom_from_base_at_map_start,
                    ),
                )
            elif (
                    self.phase == "map_changed"
                    and transform.header.frame_id == "map"):
                self.record(
                    self.from_message(transform),
                    compose(self.fixed_map_from_world, self.world_from_base_later),
                )

    def record(self, actual, expected):
        """Advance after repeated matches and fail on persistent disagreement."""
        if close_transform(actual, expected):
            self.matches += 1
        else:
            self.mismatches += 1
        if self.mismatches >= 10:
            self.fail(
                f"{self.phase} transform disagreed with fixed alignment "
                f"{self.mismatches} times")
            return
        required = 5 if self.phase != "map_changed" else 10
        if self.matches < required:
            return
        if self.phase == "odom":
            self.advance("map_initial")
        elif self.phase == "map_initial":
            self.advance("map_changed")
        else:
            self.done = True
            self.get_logger().info(
                "Ground-truth display PASSED: nonzero spawn aligned and later "
                "map->odom correction did not move truth")

    def advance(self, phase):
        """Begin the next probe phase."""
        self.phase = phase
        self.phase_started = time.monotonic()
        self.matches = 0
        self.mismatches = 0

    def fail(self, reason):
        """Stop the probe with a failure reason."""
        self.error = reason
        self.done = True
        self.get_logger().error(reason)


def main():
    rclpy.init()
    node = GroundTruthDisplayProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 1 if node.error else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
