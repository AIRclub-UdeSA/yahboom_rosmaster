#!/usr/bin/env python3
"""Measure strafe divergence between wheel odometry and ground truth."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def stamp_seconds(stamp):
    """Convert a ROS time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_yaw(orientation):
    """Return planar yaw from a normalized geometry_msgs quaternion."""
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def relative_translation(start, end):
    """Express an odometry position delta in its initial body coordinates."""
    dx = end.pose.pose.position.x - start.pose.pose.position.x
    dy = end.pose.pose.position.y - start.pose.pose.position.y
    yaw = quaternion_yaw(start.pose.pose.orientation)
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
    )


class MotionProfileDivergenceProbe(Node):
    """Command one repeatable strafe and evaluate its settled endpoint error."""

    def __init__(self):
        super().__init__("motion_profile_divergence_probe")
        self.declare_parameter("profile", "stress")
        self.declare_parameter("timeout", 40.0)
        self.declare_parameter("command_speed", 0.2)
        self.declare_parameter("command_duration", 3.0)
        self.declare_parameter("initial_settle_duration", 1.0)
        self.declare_parameter("final_settle_duration", 1.0)
        self.declare_parameter("meaningful_motion", 0.3)
        self.declare_parameter("min_translation_error", 0.003)
        self.declare_parameter("max_translation_error", 0.030)

        self.profile = str(self.get_parameter("profile").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.command_speed = float(self.get_parameter("command_speed").value)
        self.command_duration = float(
            self.get_parameter("command_duration").value)
        self.initial_settle_duration = float(
            self.get_parameter("initial_settle_duration").value)
        self.final_settle_duration = float(
            self.get_parameter("final_settle_duration").value)
        self.meaningful_motion = float(
            self.get_parameter("meaningful_motion").value)
        self.min_translation_error = float(
            self.get_parameter("min_translation_error").value)
        self.max_translation_error = float(
            self.get_parameter("max_translation_error").value)

        self.clock_time = None
        self.odometry = None
        self.ground_truth = None

        best_effort_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            lambda message: setattr(
                self, "clock_time", stamp_seconds(message.clock)),
            best_effort_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            lambda message: setattr(self, "odometry", message),
            best_effort_qos,
        )
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            lambda message: setattr(self, "ground_truth", message),
            best_effort_qos,
        )

    def ready(self):
        """Return whether command and both pose paths are active."""
        return (
            self.command_publisher.get_subscription_count() > 0
            and self.clock_time is not None
            and self.odometry is not None
            and self.ground_truth is not None
        )

    def publish_until_sim_time(self, command, duration, wall_deadline):
        """Continuously publish a command for an exact simulation-time duration."""
        if self.clock_time is None:
            return False
        target_time = self.clock_time + duration
        next_publish = 0.0
        while (
                rclpy.ok()
                and time.monotonic() < wall_deadline
                and (self.clock_time is None or self.clock_time < target_time)):
            now = time.monotonic()
            if now >= next_publish:
                self.command_publisher.publish(command)
                next_publish = now + 0.04
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.clock_time is not None and self.clock_time >= target_time

    def wait_for_fresh_endpoints(self, target_time, wall_deadline):
        """Wait until both endpoint messages have reached settled simulation time."""
        while rclpy.ok() and time.monotonic() < wall_deadline:
            odom_stamp = (
                stamp_seconds(self.odometry.header.stamp)
                if self.odometry is not None else -math.inf
            )
            truth_stamp = (
                stamp_seconds(self.ground_truth.header.stamp)
                if self.ground_truth is not None else -math.inf
            )
            if odom_stamp >= target_time - 0.05 and truth_stamp >= target_time - 0.05:
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return False

    def stop_for_wall_time(self, duration):
        """Publish final zeros even if simulation time has stopped."""
        stop = Twist()
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.command_publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

    def evaluate(self, start_odom, start_truth, end_odom, end_truth):
        """Return errors and a compact result string for the completed motion."""
        errors = []
        odom_delta = relative_translation(start_odom, end_odom)
        truth_delta = relative_translation(start_truth, end_truth)
        odom_distance = math.hypot(*odom_delta)
        truth_distance = math.hypot(*truth_delta)
        translation_error = math.hypot(
            odom_delta[0] - truth_delta[0],
            odom_delta[1] - truth_delta[1],
        )

        values = (*odom_delta, *truth_delta, translation_error)
        if not all(math.isfinite(value) for value in values):
            errors.append("motion result contains non-finite values")
        if odom_distance < self.meaningful_motion:
            errors.append(
                f"wheel odometry moved only {odom_distance:.6f}m; expected at "
                f"least {self.meaningful_motion:.3f}m")
        if truth_distance < self.meaningful_motion:
            errors.append(
                f"ground truth moved only {truth_distance:.6f}m; expected at "
                f"least {self.meaningful_motion:.3f}m")
        if not (
                self.min_translation_error
                <= translation_error
                <= self.max_translation_error):
            errors.append(
                f"translation error {translation_error:.6f}m is outside "
                f"[{self.min_translation_error:.6f}, "
                f"{self.max_translation_error:.6f}]m")

        summary = (
            f"profile={self.profile}, "
            f"odom_delta=({odom_delta[0]:.6f}, {odom_delta[1]:.6f})m, "
            f"truth_delta=({truth_delta[0]:.6f}, {truth_delta[1]:.6f})m, "
            f"translation_error={translation_error:.6f}m"
        )
        return errors, summary


def main():
    rclpy.init()
    node = MotionProfileDivergenceProbe()
    stop = Twist()
    strafe = Twist()
    strafe.linear.y = node.command_speed
    wall_deadline = time.monotonic() + node.timeout

    try:
        while rclpy.ok() and time.monotonic() < wall_deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            node.get_logger().error(
                "Motion-profile divergence FAILED: command, clock, odom, or "
                "ground-truth path did not start")
            return 1

        if not node.publish_until_sim_time(
                stop, node.initial_settle_duration, wall_deadline):
            node.get_logger().error(
                "Motion-profile divergence FAILED: initial settling timed out")
            return 1
        initial_time = node.clock_time
        if not node.wait_for_fresh_endpoints(initial_time, wall_deadline):
            node.get_logger().error(
                "Motion-profile divergence FAILED: initial poses were stale")
            return 1
        start_odom = node.odometry
        start_truth = node.ground_truth

        if not node.publish_until_sim_time(
                strafe, node.command_duration, wall_deadline):
            node.get_logger().error(
                "Motion-profile divergence FAILED: strafe command timed out")
            return 1
        if not node.publish_until_sim_time(
                stop, node.final_settle_duration, wall_deadline):
            node.get_logger().error(
                "Motion-profile divergence FAILED: final settling timed out")
            return 1
        final_time = node.clock_time
        if not node.wait_for_fresh_endpoints(final_time, wall_deadline):
            node.get_logger().error(
                "Motion-profile divergence FAILED: final poses were stale")
            return 1

        errors, summary = node.evaluate(
            start_odom, start_truth, node.odometry, node.ground_truth)
        if errors:
            node.get_logger().error(
                "Motion-profile divergence FAILED: "
                + summary + "; " + "; ".join(errors))
            return 1
        node.get_logger().info("Motion-profile divergence PASSED: " + summary)
        return 0
    finally:
        if rclpy.ok():
            node.stop_for_wall_time(0.25)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
