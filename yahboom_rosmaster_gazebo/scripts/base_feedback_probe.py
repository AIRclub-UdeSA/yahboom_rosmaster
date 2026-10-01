#!/usr/bin/env python3
"""Exercise mecanum commands and validate wheel-state odometry and TF feedback."""

import math
import os
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from real_robot_contract import RealRobotContract  # noqa: E402

# Phase lengths are in simulation time, read from the joint-state stamps, so the
# number of samples a phase collects does not depend on the real-time factor.
# /joint_states and /odom arrive at 10 Hz (#43 step 8a): settling 0.55 s then
# collecting 1.95 s gives 19 or 20 messages in the settled window, against the 10
# the checks require and the 18 the window is sized for. A wall-clock guard
# stops a phase if the simulation stalls.
SETTLE_S = 0.55
PHASE_S = 2.5
OVERSPEED_PHASE_S = 1.6
STOP_S = 0.8
WALL_GUARD_S = 40.0

# An over-limit command: x and yaw far above the limits, y below its own.
OVERSPEED_X = 3.0
OVERSPEED_Y = 0.5
OVERSPEED_Z = 9.0

WHEEL_NAMES = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
)


def stamp_key(stamp):
    """Return an exact, hashable ROS timestamp."""
    return (int(stamp.sec), int(stamp.nanosec))


def stamp_seconds(stamp):
    """Return a ROS timestamp as seconds."""
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


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
        # What the watchdog hands to Gazebo, to see the clamp on the wire.
        self.gz_subscription = self.create_subscription(
            Twist, "/cmd_vel_gz", self.capture_gz_command, 50)
        self.gz_commands = []

        self.current_phase = None
        self.phase_started = 0.0
        self.sim_now = None
        self.joint_messages = []
        self.odom_messages = []
        self.transforms = {}
        self.phase_joints = {}
        self.phase_odometry = {}

    def capture_joint_state(self, message):
        """Capture joint feedback globally and after each command settles."""
        self.sim_now = stamp_seconds(message.header.stamp)
        self.joint_messages.append(message)
        if self._phase_is_settled():
            self.phase_joints.setdefault(self.current_phase, []).append(message)

    def capture_odometry(self, message):
        """Capture odometry globally and after each command settles."""
        self.odom_messages.append(message)
        if self._phase_is_settled():
            self.phase_odometry.setdefault(self.current_phase, []).append(message)

    def capture_gz_command(self, message):
        """Capture the commands Gazebo receives while the current phase settles."""
        if self._phase_is_settled():
            self.gz_commands.append((self.current_phase, message))

    def capture_tf(self, message):
        """Index odom-to-base transforms by their exact source timestamp."""
        for transform in message.transforms:
            if (transform.header.frame_id == "odom" and
                    transform.child_frame_id == "base_footprint"):
                self.transforms[stamp_key(transform.header.stamp)] = transform

    def _phase_is_settled(self):
        return (
            self.current_phase is not None and
            self.phase_started is not None and
            self.sim_now is not None and
            self.sim_now - self.phase_started >= SETTLE_S
        )

    def ready(self):
        """Return whether the complete base-feedback path is active."""
        return bool(
            self.command_publisher.get_subscription_count() and
            self.joint_messages and self.odom_messages and self.transforms
        )

    def sim_elapsed(self, since):
        """Return the simulation time passed since ``since``, 0 before any stamp."""
        if since is None or self.sim_now is None:
            return 0.0
        return self.sim_now - since

    def publish_for(self, phase, command, duration=PHASE_S):
        """Publish a command for ``duration`` of simulation time, collecting feedback."""
        self.current_phase = phase
        self.phase_started = self.sim_now
        self.phase_joints[phase] = []
        self.phase_odometry[phase] = []
        guard = time.monotonic() + WALL_GUARD_S
        next_publish = 0.0
        while (rclpy.ok() and time.monotonic() < guard and
               self.sim_elapsed(self.phase_started) < duration):
            now = time.monotonic()
            if now >= next_publish:
                self.command_publisher.publish(command)
                next_publish = now + 0.04
            rclpy.spin_once(self, timeout_sec=0.02)

        self.current_phase = None
        self.stop_for(STOP_S)

    def stop_for(self, duration, guard_s=WALL_GUARD_S):
        """Publish zero for ``duration`` of simulation time to separate phases."""
        stop = Twist()
        started = self.sim_now
        guard = time.monotonic() + guard_s
        while (rclpy.ok() and time.monotonic() < guard and
               self.sim_elapsed(started) < duration):
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

    def validate_command_limits(self, errors):
        """Require an over-limit command to reach Gazebo saturated per component."""
        contract = RealRobotContract.load()
        x_limit = contract.physical("command.linear_x_limit_mps")
        z_limit = contract.physical("command.angular_z_limit_rad_s")
        received = [
            message for phase, message in self.gz_commands if phase == "overspeed"]
        if len(received) < 5:
            errors.append(f"command limits: only {len(received)} settled commands")
            return
        # The command was (OVERSPEED_X, OVERSPEED_Y, OVERSPEED_Z), over the x and
        # yaw limits and under the y limit: x and yaw saturate, y is untouched,
        # which a vector scaling would not do.
        for label, values, expected in (
                ("linear x", [m.linear.x for m in received], x_limit),
                ("linear y", [m.linear.y for m in received], OVERSPEED_Y),
                ("angular z", [m.angular.z for m in received], z_limit)):
            if any(abs(value - expected) > 1e-9 for value in values):
                errors.append(
                    f"command limits: {label} reached Gazebo as "
                    f"{min(values):.6f}..{max(values):.6f}, expected {expected}")
        speeds = [
            message.twist.twist.linear.x
            for message in self.phase_odometry.get("overspeed", [])
            if math.isfinite(message.twist.twist.linear.x)]
        if not speeds or statistics.median(speeds) > 1.1 * x_limit:
            errors.append(
                f"command limits: median odometry speed "
                f"{statistics.median(speeds) if speeds else math.nan:.3f} m/s "
                f"exceeds {x_limit} m/s")

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
        self.validate_command_limits(errors)
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
        node.publish_for(
            "overspeed", command(OVERSPEED_X, OVERSPEED_Y, OVERSPEED_Z),
            duration=OVERSPEED_PHASE_S)
        errors = node.validate()
        if errors:
            node.get_logger().error("Base feedback FAILED: " + "; ".join(errors))
            return 1
        node.get_logger().info(
            "Base feedback PASSED: positive x/y/yaw wheel signs, odometry, "
            "timestamps, odom->base_footprint TF and command limits agree")
        return 0
    finally:
        if rclpy.ok():
            node.stop_for(0.25, guard_s=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
