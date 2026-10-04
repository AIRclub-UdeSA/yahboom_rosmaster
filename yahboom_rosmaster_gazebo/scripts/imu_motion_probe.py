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

from real_robot_contract import RealRobotContract
from sensor_profiles import default_path as profiles_path, load_sensor_profile


# imu_link is mounted at rpy (0, pi, pi/2) relative to base_link (real-robot
# mount parity; see real_robot_contract.yaml). That rotation is a signed-axis
# permutation: body +X reads on sensor -Y, body +Y reads on sensor -X, and
# body +Z reads on sensor -Z (confirmed empirically for +Z via stationary
# gravity, which lands on accel.z ~= -9.8). All IMU vector fields -- linear
# acceleration, angular velocity, and orientation-derived yaw -- are reported
# in this same rotated sensor frame, so every body-frame check below reads
# the mapped sensor axis with its sign flipped back to body-frame terms.
BODY_TO_SENSOR_LINEAR_AXIS = {"x": "y", "y": "x"}


# /imu/data's orientation is imu_filter_madgwick's estimate of imu_link in the
# ENU world (#43 step 8c). imu_link is mounted upside down and turned a quarter
# turn on base_link, so its roll sits near +-pi and its yaw is arbitrary and drifts
# with gyro noise: the filter starts from the first accelerometer sample, and
# without a magnetometer yaw is unobservable. The probe therefore expresses the
# estimate in base_link through TF before it judges roll and pitch (near zero on
# a flat floor) and uses yaw only as a change.
NUMERIC_TOLERANCE = 1e-9
FLAT_FLOOR_TILT_RAD = 0.1


def quaternion_multiply(first, second):
    """Return the Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(quaternion):
    """Return the inverse of a unit (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_tuple(quaternion):
    """Return a geometry_msgs quaternion as an (x, y, z, w) tuple."""
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


def tuple_rpy(quaternion):
    """Return roll, pitch and yaw of an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


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
        # Which sensor profile the simulator runs, for the noise it must deliver,
        # and how many stationary raw samples to grade it on (0 skips the noise).
        self.declare_parameter("sensor_profile", "physical")
        self.declare_parameter("noise_samples", 0)
        # Grade the stationary phase and the noise only; no motion.
        self.declare_parameter("skip_motion", False)

        self.timeout = float(self.get_parameter("timeout").value)
        self.stationary_samples = max(
            10, int(self.get_parameter("stationary_samples").value))
        self.warmup_samples = max(0, int(self.get_parameter("warmup_samples").value))
        self.nominal_rate = float(self.get_parameter("nominal_rate").value)
        self.linear_command = float(self.get_parameter("linear_command").value)
        self.linear_duration = float(self.get_parameter("linear_duration").value)
        self.yaw_command = float(self.get_parameter("yaw_command").value)
        self.yaw_duration = float(self.get_parameter("yaw_duration").value)
        self.sensor_profile = str(self.get_parameter("sensor_profile").value)
        self.noise_samples = int(self.get_parameter("noise_samples").value)
        self.skip_motion = bool(self.get_parameter("skip_motion").value)

        # Every expectation below is read from the parity ledger, not copied here.
        contract = RealRobotContract.load()
        self.orientation_source = contract.nominal("imu.orientation_source")
        self.raw_topic = contract.nominal("imu.raw_topic")
        self.orientation_variance = contract.nominal(
            "imu.orientation_covariance_diag")
        self.angular_covariance = contract.nominal(
            "imu.angular_velocity_covariance_diag")
        self.linear_covariance = contract.nominal(
            "imu.linear_acceleration_covariance_diag")
        # The physical profile's noise is the ledger's; the ideal profile has none.
        physical = contract.nominal("imu.gyro_noise_stddev_rad_s"), contract.nominal(
            "imu.accel_noise_stddev_mps2")
        profile = load_sensor_profile(profiles_path(), "imu", self.sensor_profile)
        self.gyro_noise = (
            physical[0] if self.sensor_profile == "physical"
            else profile["gyro_noise_stddev_rad_s"])
        self.accel_noise = (
            physical[1] if self.sensor_profile == "physical"
            else profile["accel_noise_stddev_mps2"])
        # With no noise configured, what is left is the sensor's own jitter: it
        # must stay below a hundredth of the smallest physical figure.
        self.noise_floor = {
            "gyro": 0.01 * min(physical[0]), "accel": 0.01 * min(physical[1])}

        sensor_qos = QoSProfile(
            depth=max(50, self.stationary_samples + self.warmup_samples),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.messages = []
        self.raw_messages = []
        self.subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, sensor_qos)
        self.raw_subscription = self.create_subscription(
            Imu, self.raw_topic, self.raw_callback, sensor_qos)
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def imu_callback(self, message):
        """Retain every message needed for phase-specific validation."""
        self.messages.append(message)

    def raw_callback(self, message):
        """Retain the raw stream, to grade its content and its noise."""
        self.raw_messages.append(message)

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
    def validate_diagonal_covariance(label, covariance, diagonal, errors):
        """Require a covariance matrix to be exactly the ledger's diagonal."""
        expected = [0.0] * 9
        for index, value in enumerate(diagonal):
            expected[index * 4] = value
        if len(covariance) != 9 or any(
                abs(float(value) - want) > 1e-12
                for value, want in zip(covariance, expected)):
            errors.append(
                f"{label} covariance is {list(map(float, covariance))}, expected "
                f"the ledger's diagonal {list(diagonal)}")
            return "differs from the ledger"
        return "matches the ledger"

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

    def validate_public_interface(self, errors):
        """Require one publisher per IMU topic, the nodes and QoS the robot has."""
        identities = {}
        for topic, node_name, reliable in (
                ("/imu/data", self.orientation_source, True),
                (self.raw_topic, "imu_raw_relay", True)):
            publishers = self.get_publishers_info_by_topic(topic)
            names = [
                f"{publisher.node_namespace}{publisher.node_name}"
                for publisher in publishers]
            if len(publishers) != 1:
                errors.append(
                    f"expected exactly one {topic} publisher ({node_name}), "
                    f"found {len(publishers)}: {names}")
                continue
            publisher = publishers[0]
            identities[topic] = publisher.node_name
            if publisher.node_name != node_name:
                errors.append(
                    f"expected {topic} publisher node {node_name}, got "
                    f"{publisher.node_namespace}{publisher.node_name}")
            if publisher.topic_type != "sensor_msgs/msg/Imu":
                errors.append(
                    f"expected {topic} type sensor_msgs/msg/Imu, got "
                    f"{publisher.topic_type}")
            if reliable and (
                    publisher.qos_profile.reliability != ReliabilityPolicy.RELIABLE):
                errors.append(
                    f"expected {topic} to publish Reliable, as the robot's does, "
                    f"got {publisher.qos_profile.reliability.name}")
        return ", ".join(f"{topic}={name}" for topic, name in identities.items())

    def base_link_orientation(self, message):
        """Return the base_link orientation in the world from one /imu/data message."""
        transform = self.tf_buffer.lookup_transform(
            "base_link", "imu_link", Time(), timeout=Duration(seconds=1.0))
        imu_in_base = quaternion_tuple(transform.transform.rotation)
        imu_in_world = quaternion_tuple(message.orientation)
        return quaternion_multiply(imu_in_world, quaternion_conjugate(imu_in_base))

    def validate_raw_stream(self, messages, errors):
        """Require /imu/data_raw to carry exactly what the physical driver publishes."""
        if not messages:
            errors.append(f"no {self.raw_topic} messages arrived")
            return "no raw messages"
        for message in messages:
            quaternion = quaternion_tuple(message.orientation)
            if quaternion != (0.0, 0.0, 0.0, 1.0):
                errors.append(
                    f"{self.raw_topic} orientation is {quaternion}, the identity "
                    "the driver leaves in it")
                break
        for label, covariance in (
                ("orientation", messages[-1].orientation_covariance),
                ("angular velocity", messages[-1].angular_velocity_covariance),
                ("linear acceleration", messages[-1].linear_acceleration_covariance)):
            if any(value != 0.0 for value in covariance):
                errors.append(
                    f"{self.raw_topic} {label} covariance is not all zero")
        if any(message.header.frame_id != "imu_link" for message in messages):
            errors.append(f"{self.raw_topic} frame is not imu_link")
        if any(not self.finite_message(message) for message in messages):
            errors.append(f"{self.raw_topic} contains non-finite values")
        return f"{len(messages)} raw messages"

    def validate_noise(self, messages, errors):
        """
        Grade the stationary per-axis noise of the raw stream against the profile.

        The standard error of a standard deviation estimated from N samples is
        sigma / sqrt(2 (N - 1)); the check allows four of them. A profile with no
        noise must show none: the sensor's own jitter is far below the physical
        floor, so a hundredth of the smallest physical figure is the bound.
        """
        if len(messages) < 3:
            errors.append("too few raw samples to grade the noise")
            return "no noise statistics"
        count = len(messages)
        relative = 4.0 / math.sqrt(2.0 * (count - 1))
        axes = ("x", "y", "z")
        report = []
        for label, field, expected in (
                ("gyro", "angular_velocity", self.gyro_noise),
                ("accel", "linear_acceleration", self.accel_noise)):
            for axis, sigma in zip(axes, expected):
                values = [getattr(getattr(m, field), axis) for m in messages]
                measured = statistics.stdev(values)
                report.append(f"{label}_{axis}={measured:.4f}")
                if sigma == 0.0:
                    if measured > self.noise_floor[label]:
                        errors.append(
                            f"{label} {axis} noise {measured:.6f} under the ideal "
                            "profile, which has none")
                elif abs(measured - sigma) > relative * sigma:
                    errors.append(
                        f"{label} {axis} noise {measured:.5f} is not within "
                        f"{relative:.0%} of {sigma} (N={count})")
        return f"N={count}, " + ", ".join(report)

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
        if acceleration_z < -11.5 or acceleration_z > -8.0:
            errors.append(
                f"stationary gravity must point -Z in imu_link; median az={acceleration_z:.3f}")
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
        # Roll and pitch of base_link on a flat floor, from the filter's estimate of
        # imu_link and the static mount. The filter starts from the first
        # accelerometer sample, so there is no settling to wait for: the bound is
        # seven standard deviations of the accelerometer's noise as an angle
        # (0.135 m/s^2 / 9.76 m/s^2 = 0.014 rad), not the former 0.35.
        try:
            base_rpy = [
                tuple_rpy(self.base_link_orientation(message)) for message in messages]
            roll = statistics.median(value[0] for value in base_rpy)
            pitch = statistics.median(value[1] for value in base_rpy)
            if abs(roll) > FLAT_FLOOR_TILT_RAD or abs(pitch) > FLAT_FLOOR_TILT_RAD:
                errors.append(
                    "flat-world orientation is inconsistent in base_link: "
                    f"roll={roll:.3f}, pitch={pitch:.3f}")
        except TransformException as error:
            roll = pitch = math.nan
            errors.append(f"base_link -> imu_link does not resolve: {error}")

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
            "orientation": self.validate_diagonal_covariance(
                "orientation", messages[-1].orientation_covariance,
                self.orientation_variance, errors),
            "angular_velocity": self.validate_diagonal_covariance(
                "angular velocity", messages[-1].angular_velocity_covariance,
                self.angular_covariance, errors),
            "linear_acceleration": self.validate_diagonal_covariance(
                "linear acceleration", messages[-1].linear_acceleration_covariance,
                self.linear_covariance, errors),
        }

        publisher = self.validate_public_interface(errors)
        raw_window = [
            message for message in self.raw_messages
            if stamps[0] <= stamp_seconds(message) <= stamps[-1]]
        raw_report = self.validate_raw_stream(raw_window, errors)

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
            f"roll={roll:.3f}, pitch={pitch:.3f}, "
            f"publisher={publisher}, covariance={covariance_states}, "
            f"raw={raw_report}"
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
        sensor_axis = BODY_TO_SENSOR_LINEAR_AXIS[axis]
        values = [
            -getattr(message.linear_acceleration, sensor_axis)
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
        # body +Z (yaw) reads on sensor -Z; see BODY_TO_SENSOR_LINEAR_AXIS.
        yaw_rates = [-message.angular_velocity.z for message in steady_messages]
        median_yaw_rate = statistics.median(yaw_rates)
        positive_fraction = sum(rate > 0.05 for rate in yaw_rates) / len(yaw_rates)
        if median_yaw_rate < 0.15:
            errors.append(
                f"positive /cmd_vel produced median wz={median_yaw_rate:.3f} rad/s")
        if positive_fraction < 0.75:
            errors.append(
                f"only {positive_fraction:.0%} of steady yaw samples have positive wz")

        # The orientation is the filter's estimate of imu_link, whose absolute yaw
        # is arbitrary (see the top of this file). Express it in base_link and judge
        # only the change of yaw, which the gyro drives.
        try:
            yaws = [tuple_rpy(self.base_link_orientation(stationary_last))[2]]
            yaws.extend(
                tuple_rpy(self.base_link_orientation(message))[2]
                for message in messages)
        except TransformException as error:
            errors.append(f"base_link -> imu_link does not resolve: {error}")
            yaws = [0.0, 0.0]
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
        required = node.warmup_samples + max(
            node.stationary_samples, node.noise_samples)
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

            if node.noise_samples:
                # The raw stream's own samples, after the same warm-up.
                raw = node.raw_messages[
                    node.warmup_samples:node.warmup_samples + node.noise_samples]
                noise_errors = []
                noise_report = node.validate_noise(raw, noise_errors)
                node.get_logger().info(
                    f"Stationary {node.sensor_profile} noise: " + noise_report)
                errors.extend(noise_errors)

            if node.skip_motion:
                pass
            elif not stationary_errors and time.monotonic() < deadline:
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

            if not node.skip_motion and not stationary_errors and time.monotonic() < deadline:
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
