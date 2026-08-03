#!/usr/bin/env python3
"""Validate the simulator's ROS ground-truth odometry contract."""

from collections import deque
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


GROUND_TRUTH_TOPIC = "/ground_truth/odom"


def stamp_seconds(stamp):
    """Convert a ROS time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GroundTruthContractProbe(Node):
    """Collect ground truth, clock, and TF samples and validate their contract."""

    def __init__(self):
        super().__init__("ground_truth_contract_probe")
        self.declare_parameter("timeout", 30.0)
        self.declare_parameter("samples", 10)
        self.declare_parameter("clock_tolerance", 0.1)
        self.timeout = float(self.get_parameter("timeout").value)
        self.samples = max(2, int(self.get_parameter("samples").value))
        self.clock_tolerance = float(
            self.get_parameter("clock_tolerance").value)

        self.ground_truth = []
        self.clock_times = deque(maxlen=5000)
        self.dynamic_tf = []
        self.static_tf = []
        self._subscription_handles = []

        best_effort_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._subscription_handles.extend([
            self.create_subscription(
                Odometry,
                GROUND_TRUTH_TOPIC,
                self.capture_ground_truth,
                best_effort_qos,
            ),
            self.create_subscription(
                Clock,
                "/clock",
                lambda message: self.clock_times.append(
                    stamp_seconds(message.clock)),
                best_effort_qos,
            ),
            self.create_subscription(
                TFMessage,
                "/tf",
                lambda message: self.dynamic_tf.append(message),
                best_effort_qos,
            ),
            self.create_subscription(
                TFMessage,
                "/tf_static",
                lambda message: self.static_tf.append(message),
                static_tf_qos,
            ),
        ])

        self.get_logger().info(
            f"Waiting up to {self.timeout:.1f}s for {GROUND_TRUTH_TOPIC}")

    def capture_ground_truth(self, message):
        """Keep the requested number of ground-truth samples."""
        if len(self.ground_truth) < self.samples:
            self.ground_truth.append(message)

    def complete(self):
        """Return whether enough data exists for every contract check."""
        return (
            len(self.ground_truth) >= self.samples
            and len(self.clock_times) >= self.samples
            and bool(self.dynamic_tf)
            and bool(self.static_tf)
        )

    @staticmethod
    def finite(values):
        """Return whether all values are finite numbers."""
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def normalized_frame(frame):
        """Normalize a frame ID for authority checks only."""
        return frame.lstrip("/")

    def validate(self):
        """Return all observed ground-truth contract violations."""
        errors = []
        if len(self.ground_truth) < self.samples:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: received "
                f"{len(self.ground_truth)}/{self.samples} messages")
        if len(self.clock_times) < self.samples:
            errors.append(
                f"/clock: received {len(self.clock_times)}/{self.samples} messages")
        if not self.dynamic_tf:
            errors.append("/tf: no messages received")
        if not self.static_tf:
            errors.append("/tf_static: no messages received")
        if errors:
            return errors

        publishers = self.get_publishers_info_by_topic(GROUND_TRUTH_TOPIC)
        if len(publishers) != 1:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: expected exactly one publisher, "
                f"found {len(publishers)}")

        ground_truth_stamps = [
            stamp_seconds(message.header.stamp)
            for message in self.ground_truth
        ]
        if any(stamp <= 0.0 for stamp in ground_truth_stamps):
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: timestamp is zero or negative")
        if any(
                current <= previous
                for previous, current in zip(
                    ground_truth_stamps, ground_truth_stamps[1:])):
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: timestamps are not strictly increasing")

        clock_times = list(self.clock_times)
        for stamp in ground_truth_stamps:
            nearest_clock_error = min(
                abs(clock_time - stamp) for clock_time in clock_times)
            if nearest_clock_error > self.clock_tolerance:
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: stamp {stamp:.9f} is "
                    f"{nearest_clock_error:.6f}s from the nearest /clock sample")
                break

        for message in self.ground_truth:
            if message.header.frame_id != "world":
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: expected frame world, got "
                    f"{message.header.frame_id!r}")
                break
        for message in self.ground_truth:
            if message.child_frame_id != "base_footprint":
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: expected child base_footprint, got "
                    f"{message.child_frame_id!r}")
                break

        for message in self.ground_truth:
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            pose_values = (
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            if not self.finite(pose_values):
                errors.append(f"{GROUND_TRUTH_TOPIC}: pose contains non-finite values")
                break
            quaternion_norm = math.sqrt(
                orientation.x ** 2
                + orientation.y ** 2
                + orientation.z ** 2
                + orientation.w ** 2
            )
            if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-4):
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: orientation norm is "
                    f"{quaternion_norm:.9f}, expected 1")
                break

        forbidden_tf = []
        for channel, messages in (
                ("/tf", self.dynamic_tf), ("/tf_static", self.static_tf)):
            for message in messages:
                for transform in message.transforms:
                    parent = self.normalized_frame(transform.header.frame_id)
                    child = self.normalized_frame(transform.child_frame_id)
                    if parent == "world" and child in {
                            "base_footprint", "base_link"}:
                        forbidden_tf.append(f"{channel}: {parent} -> {child}")
        if forbidden_tf:
            errors.append(
                "ground truth must not publish robot TF: "
                + ", ".join(sorted(set(forbidden_tf))))

        return errors


def main():
    rclpy.init()
    node = GroundTruthContractProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)

        errors = node.validate()
        if errors:
            node.get_logger().error(
                "Ground-truth contract FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info(
            "Ground-truth contract PASSED: "
            f"ground_truth={len(node.ground_truth)}, "
            f"clock={len(node.clock_times)}, "
            f"tf={len(node.dynamic_tf)}, tf_static={len(node.static_tf)}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
