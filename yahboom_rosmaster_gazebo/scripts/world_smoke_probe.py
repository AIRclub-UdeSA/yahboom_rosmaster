#!/usr/bin/env python3
"""Smoke-test a practice world: valid spawn, no initial collision, essential interfaces.

Unlike sensor_contract_probe.py (which validates rates and message shape on the
two reference worlds), this probe is world-agnostic: it only checks that the
robot settles upright and stationary right after spawn -- the one thing that
actually varies across worlds -- and that the core topics are alive.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage


GROUND_TRUTH_TOPIC = "/ground_truth/odom"
REQUIRED_DYNAMIC_TF_EDGE = ("odom", "base_footprint")
MAX_SETTLED_DRIFT_M = 0.05
MAX_LEVEL_ANGLE_RAD = 0.15


def stamp_seconds(stamp):
    """Convert a ROS time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_roll_pitch(orientation):
    """Convert a quaternion to roll and pitch in radians, ignoring yaw."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


class WorldSmokeProbe(Node):
    """Confirm a practice world spawns the robot upright, settled, and publishing."""

    def __init__(self):
        super().__init__("world_smoke_probe")
        self.declare_parameter("timeout", 25.0)
        self.declare_parameter("settle_window", 3.0)
        self.timeout = float(self.get_parameter("timeout").value)
        self.settle_window = float(self.get_parameter("settle_window").value)

        self.ground_truth = []
        self.essential_seen = {
            "/odom": False,
            "/tf": False,
            "/joint_states": False,
            "/scan": False,
            "/imu/data": False,
        }
        self._subscription_handles = []

        best_effort_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        default_qos = QoSProfile(depth=20)

        self._subscription_handles.append(self.create_subscription(
            Odometry, GROUND_TRUTH_TOPIC, self.capture_ground_truth, best_effort_qos))
        self._subscription_handles.append(self.create_subscription(
            Odometry, "/odom", lambda message: self.mark("/odom"), default_qos))
        self._subscription_handles.append(self.create_subscription(
            TFMessage, "/tf", self.capture_tf, default_qos))
        self._subscription_handles.append(self.create_subscription(
            JointState, "/joint_states", lambda message: self.mark("/joint_states"), default_qos))
        self._subscription_handles.append(self.create_subscription(
            LaserScan, "/scan", lambda message: self.mark("/scan"), best_effort_qos))
        self._subscription_handles.append(self.create_subscription(
            Imu, "/imu/data", lambda message: self.mark("/imu/data"), best_effort_qos))

        self.get_logger().info(
            f"Waiting up to {self.timeout:.1f}s for the world smoke contract")

    def mark(self, topic):
        """Record that at least one message arrived on an essential topic."""
        self.essential_seen[topic] = True

    def capture_tf(self, message):
        """Track whether the dynamic odom -> base_footprint edge is being published."""
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            if (parent, child) == REQUIRED_DYNAMIC_TF_EDGE:
                self.essential_seen["/tf"] = True

    def capture_ground_truth(self, message):
        """Keep ground-truth samples spanning the settle window from first arrival."""
        if not self.ground_truth:
            self.ground_truth.append(message)
            return
        elapsed = stamp_seconds(message.header.stamp) - stamp_seconds(
            self.ground_truth[0].header.stamp)
        if elapsed <= self.settle_window:
            self.ground_truth.append(message)

    def complete(self):
        """Return whether the settle window has elapsed and every topic was seen."""
        if not self.ground_truth:
            return False
        elapsed = stamp_seconds(self.ground_truth[-1].header.stamp) - stamp_seconds(
            self.ground_truth[0].header.stamp)
        return elapsed >= self.settle_window and all(self.essential_seen.values())

    def validate(self):
        """Return all observed spawn/collision/interface contract violations."""
        errors = []
        missing = [topic for topic, seen in self.essential_seen.items() if not seen]
        if missing:
            errors.append(f"missing essential interfaces: {missing}")
        if not self.ground_truth:
            errors.append(f"{GROUND_TRUTH_TOPIC}: no messages received")
            return errors

        first, last = self.ground_truth[0], self.ground_truth[-1]
        for label, message in (("first", first), ("last", last)):
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            values = (
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                errors.append(f"{GROUND_TRUTH_TOPIC}: {label} pose has non-finite values")
                continue
            roll, pitch = quaternion_to_roll_pitch(orientation)
            if abs(roll) > MAX_LEVEL_ANGLE_RAD or abs(pitch) > MAX_LEVEL_ANGLE_RAD:
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: {label} pose is tipped "
                    f"(roll={math.degrees(roll):.1f}deg, pitch={math.degrees(pitch):.1f}deg) "
                    "-- likely spawned inside or against world geometry")
        if errors:
            return errors

        drift = math.dist(
            (
                first.pose.pose.position.x,
                first.pose.pose.position.y,
                first.pose.pose.position.z,
            ),
            (
                last.pose.pose.position.x,
                last.pose.pose.position.y,
                last.pose.pose.position.z,
            ),
        )
        if drift > MAX_SETTLED_DRIFT_M:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: drifted {drift:.3f}m with no command over "
                f"{self.settle_window:.1f}s -- likely an initial collision or "
                "spawn on unstable geometry")

        return errors

    def summary(self):
        """Return a compact status summary for logging."""
        return (
            f"ground_truth={len(self.ground_truth)}, "
            + ", ".join(f"{topic}={seen}" for topic, seen in self.essential_seen.items())
        )


def main():
    rclpy.init()
    node = WorldSmokeProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        errors = node.validate()
        if errors:
            node.get_logger().error("World smoke contract FAILED: " + "; ".join(errors))
            node.get_logger().error("Received: " + node.summary())
            return 1
        node.get_logger().info("World smoke contract PASSED: " + node.summary())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
