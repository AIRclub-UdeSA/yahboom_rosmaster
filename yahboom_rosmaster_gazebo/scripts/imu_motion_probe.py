#!/usr/bin/env python3
"""Validate stationary and commanded-motion IMU semantics in the simulator."""

import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_seconds(message):
    """Return a message header stamp in seconds."""
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def quaternion_norm(quaternion):
    """Return the Euclidean norm of a geometry_msgs quaternion."""
    return math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )


def quaternion_rpy(quaternion):
    """Convert a normalized quaternion to roll, pitch, and yaw."""
    sin_roll = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cos_roll = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def shortest_angle(current, previous):
    """Return the signed shortest angular displacement current - previous."""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


class ImuMotionProbe(Node):
    """Collect IMU messages and command bounded planar maneuvers."""

    def __init__(self):
        super().__init__("imu_motion_probe")
        self.declare_parameter("timeout", 45.0)
        self.declare_parameter("stationary_samples", 20)
        self.declare_parameter("warmup_samples", 5)
        self.declare_parameter("nominal_rate", 15.0)
        self.declare_parameter("linear_command", 0.4)
        self.declare_parameter("linear_duration", 0.7)
        self.declare_parameter("yaw_command", 0.5)
        self.declare_parameter("yaw_duration", 2.0)

        self.timeout = float(self.get_parameter("timeout").value)
        self.stationary_samples = max(
            10, int(self.get_parameter("stationary_samples").value))
        self.warmup_samples = max(0, int(self.get_parameter("warmup_samples").value))
        self.nominal_rate = float(self.get_parameter("nominal_rate").value)
        self.linear_command = float(self.get_parameter("linear_command").value)
        self.linear_duration = float(self.get_parameter("linear_duration").value)
        self.yaw_command = float(self.get_parameter("yaw_command").value)
        self.yaw_duration = float(self.get_parameter("yaw_duration").value)

        sensor_qos = QoSProfile(
            depth=max(50, self.stationary_samples + self.warmup_samples),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.messages = []
        self.subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, sensor_qos)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def imu_callback(self, message):
        """Retain every message needed for phase-specific validation."""
        self.messages.append(message)

    def publish_command(self, linear_x=0.0, linear_y=0.0, yaw_rate=0.0):
        """Publish a planar velocity command."""
        command = Twist()
        command.linear.x = float(linear_x)
        command.linear.y = float(linear_y)
        command.angular.z = float(yaw_rate)
        self.command_publisher.publish(command)

    def publish_stop(self):
        """Publish zero repeatedly so the robot stops even during teardown."""
        for _ in range(12):
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_count(self, required_count, deadline):
        """Spin until the requested count arrives or the wall-clock deadline expires."""
        while (
                rclpy.ok()
                and time.monotonic() < deadline
                and len(self.messages) < required_count):
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.messages) >= required_count

    @staticmethod
    def finite_message(message):
        """Return whether all IMU measurement and orientation values are finite."""
        values = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            *message.orientation_covariance,
            *message.angular_velocity_covariance,
            *message.linear_acceleration_covariance,
        )
        return all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def validate_covariance(label, covariance, errors):
        """Validate the ROS Imu covariance sentinel or matrix convention."""
        if len(covariance) != 9:
            errors.append(f"{label} covariance has {len(covariance)} entries")
            return "invalid"
        if not all(math.isfinite(float(value)) for value in covariance):
            errors.append(f"{label} covariance contains non-finite values")
            return "invalid"
        if covariance[0] == -1.0:
            return "unavailable (-1 sentinel)"
        if all(abs(value) <= 1e-12 for value in covariance):
            return "unknown (all-zero matrix)"

        for row, column in ((0, 1), (0, 2), (1, 2)):
            first = covariance[row * 3 + column]
            second = covariance[column * 3 + row]
            if not math.isclose(first, second, rel_tol=1e-6, abs_tol=1e-12):
                errors.append(f"{label} covariance is not symmetric")
                break
        if any(covariance[index] < 0.0 for index in (0, 4, 8)):
            errors.append(f"{label} covariance has a negative diagonal")
        return "provided matrix"

    def validate_stationary(self, messages):
        """Validate stationary IMU frames, time, values, orientation, and rate."""
        errors = []
        stamps = [stamp_seconds(message) for message in messages]
        if any(stamp <= 0.0 for stamp in stamps):
            errors.append("stationary header timestamp is zero or negative")
        if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
            errors.append("stationary header timestamps are not strictly increasing")
        if any(message.header.frame_id != "imu_link" for message in messages):
            observed = sorted({message.header.frame_id for message in messages})
            errors.append(f"expected frame imu_link, observed {observed}")
        if any(not self.finite_message(message) for message in messages):
            errors.append("stationary IMU contains non-finite values")

        deltas = [current - previous for previous, current in zip(stamps, stamps[1:])]
        measured_rate = 1.0 / statistics.median(deltas) if deltas else 0.0
        if not 0.8 * self.nominal_rate <= measured_rate <= 1.2 * self.nominal_rate:
            errors.append(
                f"stationary rate {measured_rate:.2f} Hz is outside "
                f"{self.nominal_rate:.2f} Hz +/-20%")

        acceleration_x = statistics.median(
            message.linear_acceleration.x for message in messages)
        acceleration_y = statistics.median(
            message.linear_acceleration.y for message in messages)
        acceleration_z = statistics.median(
            message.linear_acceleration.z for message in messages)
        gravity_magnitudes = [
            math.sqrt(
                message.linear_acceleration.x ** 2
                + message.linear_acceleration.y ** 2
                + message.linear_acceleration.z ** 2)
            for message in messages
        ]
        gravity = statistics.median(gravity_magnitudes)
        if acceleration_z < 8.0 or acceleration_z > 11.5:
            errors.append(
                f"stationary gravity must point +Z in imu_link; median az={acceleration_z:.3f}")
        if abs(acceleration_x) > 0.75 or abs(acceleration_y) > 0.75:
            errors.append(
                "stationary horizontal acceleration is too large: "
                f"ax={acceleration_x:.3f}, ay={acceleration_y:.3f}")
        if not 8.0 <= gravity <= 11.5:
            errors.append(f"stationary gravity magnitude is {gravity:.3f} m/s^2")

        quaternion_norms = [quaternion_norm(message.orientation) for message in messages]
        if any(not 0.995 <= norm <= 1.005 for norm in quaternion_norms):
            errors.append(
                "orientation quaternion is not normalized: "
                f"range={min(quaternion_norms):.6f}..{max(quaternion_norms):.6f}")
        rpy_values = [quaternion_rpy(message.orientation) for message in messages]
        roll = statistics.median(value[0] for value in rpy_values)
        pitch = statistics.median(value[1] for value in rpy_values)
        if abs(roll) > 0.35 or abs(pitch) > 0.35:
            errors.append(
                f"flat-world orientation is inconsistent: roll={roll:.3f}, pitch={pitch:.3f}")

        angular_speed = statistics.median(
            math.sqrt(
                message.angular_velocity.x ** 2
                + message.angular_velocity.y ** 2
                + message.angular_velocity.z ** 2)
            for message in messages
        )
        if angular_speed > 0.15:
            errors.append(
                f"stationary median angular speed is {angular_speed:.3f} rad/s")

        covariance_states = {
            "orientation": self.validate_covariance(
                "orientation", messages[-1].orientation_covariance, errors),
            "angular_velocity": self.validate_covariance(
                "angular velocity", messages[-1].angular_velocity_covariance, errors),
            "linear_acceleration": self.validate_covariance(
                "linear acceleration", messages[-1].linear_acceleration_covariance, errors),
        }
        if covariance_states["orientation"].startswith("unavailable"):
            errors.append(
                "orientation is marked unavailable, so quaternion meaning "
                "cannot be tested")

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "imu_link",
                Time.from_msg(messages[-1].header.stamp),
                timeout=Duration(seconds=1.0),
            )
            transform_values = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            if not all(math.isfinite(value) for value in transform_values):
                errors.append("base_link -> imu_link transform contains non-finite values")
            if not 0.995 <= quaternion_norm(transform.transform.rotation) <= 1.005:
                errors.append("base_link -> imu_link transform rotation is not normalized")
        except TransformException as error:
            errors.append(f"imu_link does not resolve from base_link at the IMU stamp: {error}")

        diagnostics = (
            f"samples={len(messages)}, rate={measured_rate:.2f} Hz, "
            f"median accel=({acceleration_x:.3f}, {acceleration_y:.3f}, "
            f"{acceleration_z:.3f}) m/s^2, gravity={gravity:.3f} m/s^2, "
            f"median angular speed={angular_speed:.3f} rad/s, "
            f"roll={roll:.3f}, pitch={pitch:.3f}, covariance={covariance_states}"
        )
        return errors, diagnostics

    def command_positive_yaw(self, stationary_last, deadline):
        """Publish positive yaw until the requested simulation-time duration passes."""
        start_stamp = stamp_seconds(stationary_last)
        start_index = len(self.messages)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(yaw_rate=self.yaw_command)
            rclpy.spin_once(self, timeout_sec=0.025)
            if (
                    len(self.messages) > start_index
                    and stamp_seconds(self.messages[-1]) - start_stamp >= self.yaw_duration):
                break
        return self.messages[start_index:]

    def command_positive_linear(self, axis, start_message, deadline):
        """Publish one positive body-axis velocity start pulse."""
        start_stamp = stamp_seconds(start_message)
        start_index = len(self.messages)
        while rclpy.ok() and time.monotonic() < deadline:
            if axis == "x":
                self.publish_command(linear_x=self.linear_command)
            else:
                self.publish_command(linear_y=self.linear_command)
            rclpy.spin_once(self, timeout_sec=0.025)
            if (
                    len(self.messages) > start_index
                    and stamp_seconds(self.messages[-1]) - start_stamp
                    >= self.linear_duration):
                break
        return self.messages[start_index:]

    def settle(self, start_message, deadline, duration=0.9):
        """Command zero through a bounded simulation-time settling interval."""
        start_stamp = stamp_seconds(start_message)
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.025)
            if self.messages and stamp_seconds(self.messages[-1]) - start_stamp >= duration:
                return True
        return False

    @staticmethod
    def validate_positive_linear(axis, messages):
        """Validate body-axis acceleration during a positive velocity start."""
        errors = []
        if len(messages) < 7:
            return (
                [f"positive {axis} start produced only {len(messages)} IMU messages"],
                "insufficient data",
            )
        values = [
            getattr(message.linear_acceleration, axis)
            for message in messages
        ]
        positive_values = [value for value in values if value > 0.35]
        peak = max(values)
        if len(positive_values) < 3:
            errors.append(
                f"positive {axis} start produced only {len(positive_values)} "
                f"samples above +0.35 m/s^2 (peak={peak:.3f})")
        diagnostics = (
            f"samples={len(messages)}, peak a{axis}={peak:.3f} m/s^2, "
            f"positive response samples={len(positive_values)}"
        )
        return errors, diagnostics

    def validate_positive_yaw(self, stationary_last, messages):
        """Validate gyro sign and quaternion response during positive yaw."""
        errors = []
        if len(messages) < 10:
            return (
                [f"positive yaw produced only {len(messages)} IMU messages"],
                "insufficient data",
            )

        start_stamp = stamp_seconds(stationary_last)
        stamps = [stamp_seconds(message) for message in messages]
        if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
            errors.append("positive-yaw IMU timestamps are not strictly increasing")

        # Ignore the drivetrain acceleration ramp when checking steady yaw sign.
        steady_messages = [
            message for message in messages
            if stamp_seconds(message) - start_stamp >= min(0.5, self.yaw_duration / 3.0)
        ]
        if len(steady_messages) < 5:
            steady_messages = messages[len(messages) // 2:]
        yaw_rates = [message.angular_velocity.z for message in steady_messages]
        median_yaw_rate = statistics.median(yaw_rates)
        positive_fraction = sum(rate > 0.05 for rate in yaw_rates) / len(yaw_rates)
        if median_yaw_rate < 0.15:
            errors.append(
                f"positive /cmd_vel produced median wz={median_yaw_rate:.3f} rad/s")
        if positive_fraction < 0.75:
            errors.append(
                f"only {positive_fraction:.0%} of steady yaw samples have positive wz")

        yaws = [quaternion_rpy(stationary_last.orientation)[2]]
        yaws.extend(quaternion_rpy(message.orientation)[2] for message in messages)
        accumulated_yaw = sum(
            shortest_angle(current, previous)
            for previous, current in zip(yaws, yaws[1:])
        )
        if accumulated_yaw < 0.25:
            errors.append(
                f"positive /cmd_vel changed orientation yaw by only {accumulated_yaw:.3f} rad")

        diagnostics = (
            f"samples={len(messages)}, steady samples={len(steady_messages)}, "
            f"median wz={median_yaw_rate:.3f} rad/s, "
            f"positive fraction={positive_fraction:.0%}, "
            f"unwrapped orientation delta={accumulated_yaw:.3f} rad"
        )
        return errors, diagnostics


def main():
    rclpy.init()
    node = ImuMotionProbe()
    deadline = time.monotonic() + node.timeout
    errors = []
    try:
        required = node.warmup_samples + node.stationary_samples
        node.get_logger().info(
            f"Waiting for {required} stationary IMU messages on /imu/data")
        if not node.wait_for_count(required, deadline):
            errors.append(
                f"stationary phase received {len(node.messages)}/{required} messages")
        else:
            stationary = node.messages[
                node.warmup_samples:node.warmup_samples + node.stationary_samples]
            stationary_errors, stationary_diagnostics = node.validate_stationary(stationary)
            node.get_logger().info("Stationary IMU: " + stationary_diagnostics)
            errors.extend(stationary_errors)

            if not stationary_errors and time.monotonic() < deadline:
                for axis in ("x", "y"):
                    node.get_logger().info(
                        f"Commanding bounded positive {axis} start: "
                        f"v{axis}={node.linear_command:.3f} m/s for "
                        f"{node.linear_duration:.2f}s of simulation time")
                    start_message = node.messages[-1]
                    linear_messages = node.command_positive_linear(
                        axis, start_message, deadline)
                    linear_errors, linear_diagnostics = (
                        node.validate_positive_linear(axis, linear_messages))
                    node.get_logger().info(
                        f"Positive-{axis} IMU: " + linear_diagnostics)
                    errors.extend(linear_errors)
                    if linear_messages:
                        settled = node.settle(linear_messages[-1], deadline)
                        if not settled:
                            errors.append(
                                f"timed out while settling after positive {axis} start")
                            break

            if not stationary_errors and time.monotonic() < deadline:
                node.get_logger().info(
                    f"Commanding bounded positive yaw: wz={node.yaw_command:.3f} rad/s "
                    f"for {node.yaw_duration:.2f}s of simulation time")
                yaw_start = node.messages[-1]
                yaw_messages = node.command_positive_yaw(yaw_start, deadline)
                yaw_errors, yaw_diagnostics = node.validate_positive_yaw(
                    yaw_start, yaw_messages)
                node.get_logger().info("Positive-yaw IMU: " + yaw_diagnostics)
                errors.extend(yaw_errors)

        if errors:
            node.get_logger().error("IMU motion contract FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info("IMU motion contract PASSED")
        return 0
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
