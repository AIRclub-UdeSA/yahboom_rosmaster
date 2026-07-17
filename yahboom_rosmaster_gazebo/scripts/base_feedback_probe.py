#!/usr/bin/env python3
"""Exercise mecanum commands and validate wheel-state odometry and TF feedback."""

import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
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
    """Return an exact, hashable ROS timestamp."""
    return (int(stamp.sec), int(stamp.nanosec))


def finite(values):
    """Return whether every numeric value is finite."""
    return all(math.isfinite(float(value)) for value in values)


class BaseFeedbackProbe(Node):
    """Drive three body axes and compare commands, wheels, odometry, and TF."""

    def __init__(self):
        super().__init__("base_feedback_probe")
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self.capture_joint_state, 30)
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.capture_odometry, 30)
        self.tf_subscription = self.create_subscription(
            TFMessage, "/tf", self.capture_tf, 50)

        self.current_phase = None
        self.phase_started = 0.0
        self.joint_messages = []
        self.odom_messages = []
        self.transforms = {}
        self.phase_joints = {}
        self.phase_odometry = {}

    def capture_joint_state(self, message):
        """Capture joint feedback globally and after each command settles."""
        self.joint_messages.append(message)
        if self._phase_is_settled():
            self.phase_joints.setdefault(self.current_phase, []).append(message)

    def capture_odometry(self, message):
        """Capture odometry globally and after each command settles."""
        self.odom_messages.append(message)
        if self._phase_is_settled():
            self.phase_odometry.setdefault(self.current_phase, []).append(message)

    def capture_tf(self, message):
        """Index odom-to-base transforms by their exact source timestamp."""
        for transform in message.transforms:
            if (transform.header.frame_id == "odom" and
                    transform.child_frame_id == "base_footprint"):
                self.transforms[stamp_key(transform.header.stamp)] = transform

    def _phase_is_settled(self):
        return (
            self.current_phase is not None and
            time.monotonic() - self.phase_started >= 0.55
        )

    def ready(self):
        """Return whether the complete base-feedback path is active."""
        return bool(
            self.command_publisher.get_subscription_count() and
            self.joint_messages and self.odom_messages and self.transforms
        )

    def publish_for(self, phase, command, duration=1.8):
        """Publish a command continuously while callbacks collect feedback."""
        self.current_phase = phase
        self.phase_started = time.monotonic()
        self.phase_joints[phase] = []
        self.phase_odometry[phase] = []
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.command_publisher.publish(command)
                next_publish = now + 0.04
            rclpy.spin_once(self, timeout_sec=0.02)

        self.current_phase = None
        self.stop_for(0.8)

    def stop_for(self, duration):
        """Continuously publish zero long enough to separate command phases."""
        stop = Twist()
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.command_publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.04)

    @staticmethod
    def wheel_velocities(message):
        """Extract all required wheel velocities from a JointState message."""
        if len(message.velocity) < len(message.name):
            return None
        indexes = {name: index for index, name in enumerate(message.name)}
        if any(name not in indexes for name in WHEEL_NAMES):
            return None
        values = tuple(message.velocity[indexes[name]] for name in WHEEL_NAMES)
        return values if finite(values) else None

    @staticmethod
    def median_wheel_velocities(messages):
        samples = [BaseFeedbackProbe.wheel_velocities(message)
                   for message in messages]
        samples = [sample for sample in samples if sample is not None]
        if not samples:
            return None
        return tuple(statistics.median(values) for values in zip(*samples))

    @staticmethod
    def validate_stamps(label, messages, errors):
        stamps = [stamp_key(message.header.stamp) for message in messages]
        if not stamps or any(stamp == (0, 0) for stamp in stamps):
            errors.append(f"{label}: missing or zero timestamps")
            return
        nanoseconds = [sec * 1_000_000_000 + nsec for sec, nsec in stamps]
        if any(current <= previous
               for previous, current in zip(nanoseconds, nanoseconds[1:])):
            errors.append(f"{label}: timestamps are not strictly increasing")

    def validate_phase(self, phase, expected_wheel_signs, odom_component, errors):
        joints = self.phase_joints.get(phase, [])
        odometry = self.phase_odometry.get(phase, [])
        if len(joints) < 10:
            errors.append(f"{phase}: only {len(joints)} settled joint samples")
        if len(odometry) < 10:
            errors.append(f"{phase}: only {len(odometry)} settled odom samples")

        medians = self.median_wheel_velocities(joints)
        if medians is None:
            errors.append(f"{phase}: wheel velocities are absent or non-finite")
        else:
            for name, value, expected_sign in zip(
                    WHEEL_NAMES, medians, expected_wheel_signs):
                if expected_sign * value <= 0.5:
                    errors.append(
                        f"{phase}: {name} median velocity {value:.3f} has "
                        f"the wrong sign or magnitude")

        odom_values = []
        for message in odometry:
            twist = message.twist.twist
            value = {
                "linear_x": twist.linear.x,
                "linear_y": twist.linear.y,
                "angular_z": twist.angular.z,
            }[odom_component]
            if math.isfinite(value):
                odom_values.append(value)
        if not odom_values:
            errors.append(f"{phase}: odometry twist values are absent or non-finite")
        else:
            median_value = statistics.median(odom_values)
            if median_value <= 0.02:
                errors.append(
                    f"{phase}: median odometry {odom_component} "
                    f"{median_value:.3f} did not respond positively")

    def validate_feedback_chain(self, errors):
        """Validate timestamp provenance and exact odometry/TF agreement."""
        joint_stamps = {stamp_key(message.header.stamp)
                        for message in self.joint_messages}
        paired = 0
        mismatched = 0
        for odometry in self.odom_messages:
            key = stamp_key(odometry.header.stamp)
            if key not in joint_stamps:
                continue
            transform = self.transforms.get(key)
            if transform is None:
                continue
            paired += 1
            pose = odometry.pose.pose
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            differences = (
                pose.position.x - translation.x,
                pose.position.y - translation.y,
                pose.position.z - translation.z,
                pose.orientation.x - rotation.x,
                pose.orientation.y - rotation.y,
                pose.orientation.z - rotation.z,
                pose.orientation.w - rotation.w,
            )
            if any(abs(value) > 1e-9 for value in differences):
                mismatched += 1

        if paired < 20:
            errors.append(
                f"feedback chain: only {paired} exact joint/odom/TF timestamp pairs")
        elif mismatched:
            errors.append(
                f"feedback chain: {mismatched}/{paired} odom and TF poses disagree")

        for message in self.odom_messages:
            pose = message.pose.pose
            twist = message.twist.twist
            if not finite((
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                    twist.linear.x, twist.linear.y, twist.angular.z)):
                errors.append("odometry: pose or planar twist contains non-finite data")
                break

    def validate(self):
        errors = []
        if len(self.joint_messages) < 20:
            errors.append(f"joint states: received only {len(self.joint_messages)}")
        if len(self.odom_messages) < 20:
            errors.append(f"odometry: received only {len(self.odom_messages)}")
        if len(self.transforms) < 20:
            errors.append(f"odom TF: received only {len(self.transforms)} unique stamps")

        self.validate_stamps("joint states", self.joint_messages, errors)
        self.validate_stamps("odometry", self.odom_messages, errors)

        # Native Gazebo MecanumDrive wheel ordering and the odometry equations
        # use these wheel signs for positive body x, body y, and yaw.
        self.validate_phase("forward", (1, 1, 1, 1), "linear_x", errors)
        self.validate_phase("left", (-1, 1, 1, -1), "linear_y", errors)
        self.validate_phase("yaw", (-1, 1, -1, 1), "angular_z", errors)
        self.validate_feedback_chain(errors)
        return errors


def command(linear_x=0.0, linear_y=0.0, angular_z=0.0):
    message = Twist()
    message.linear.x = linear_x
    message.linear.y = linear_y
    message.angular.z = angular_z
    return message


def main():
    rclpy.init()
    node = BaseFeedbackProbe()
    try:
        ready_deadline = time.monotonic() + 25.0
        while rclpy.ok() and time.monotonic() < ready_deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            node.get_logger().error(
                "Base feedback FAILED: command, joint, odom, or TF path did not start")
            return 1

        node.publish_for("forward", command(linear_x=0.16))
        node.publish_for("left", command(linear_y=0.16))
        node.publish_for("yaw", command(angular_z=0.45))
        errors = node.validate()
        if errors:
            node.get_logger().error("Base feedback FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info(
            "Base feedback PASSED: positive x/y/yaw wheel signs, odometry, "
            "timestamps, and odom->base_footprint TF agree")
        return 0
    finally:
        if rclpy.ok():
            node.stop_for(0.25)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
