#!/usr/bin/env python3
"""
Smoke-test a practice world: valid spawn, no initial collision, essential interfaces.

Unlike sensor_contract_probe.py (which validates rates and message shape on the
two reference worlds), this probe is world-agnostic: it only checks that the
robot settles upright and stationary right after spawn, stays near its expected
spawn point, and can actually translate on a short forward command -- the
things that vary across worlds -- and that the core topics are alive.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_msgs.msg import TFMessage


GROUND_TRUTH_TOPIC = "/ground_truth/odom"
REQUIRED_DYNAMIC_TF_EDGE = ("odom", "base_footprint")
MAX_SETTLED_EXCURSION_M = 0.05
MAX_LEVEL_ANGLE_RAD = 0.15
MAX_LATERAL_DISPLACEMENT_M = 0.05
# The spawn action never sets -x/-y, so every world spawns the robot at the
# origin. A settled offset past this means the robot was pushed off spawn
# (e.g. wedged against a wall) before the probe started observing.
MAX_SPAWN_OFFSET_M = 0.10


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


def quaternion_to_yaw(orientation):
    """Convert a quaternion to yaw in radians."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WorldSmokeProbe(Node):
    """Confirm a practice world spawns the robot upright, settled, and publishing."""

    def __init__(self):
        super().__init__("world_smoke_probe")
        self.declare_parameter("timeout", 32.0)
        self.declare_parameter("settle_window", 3.0)
        self.declare_parameter("command_speed", 0.15)
        self.declare_parameter("command_duration", 2.0)
        self.declare_parameter("meaningful_motion", 0.08)
        self.timeout = float(self.get_parameter("timeout").value)
        self.settle_window = float(self.get_parameter("settle_window").value)
        self.command_speed = float(self.get_parameter("command_speed").value)
        self.command_duration = float(self.get_parameter("command_duration").value)
        self.meaningful_motion = float(self.get_parameter("meaningful_motion").value)

        self.ground_truth = []
        self.latest_ground_truth = None
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

        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

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
        """
        Track the latest sample, and keep those closing out the settle window.

        Ground truth arrives on a fixed publish period, so no sample lands on
        exactly settle_window -- stopping at the last one strictly before it
        would make the window elapsed always fall a bit short. Instead keep
        appending through the first sample that reaches or passes the window,
        then stop, so the window always closes at or after settle_window.
        """
        self.latest_ground_truth = message
        if not self.ground_truth:
            self.ground_truth.append(message)
            return
        elapsed = stamp_seconds(self.ground_truth[-1].header.stamp) - stamp_seconds(
            self.ground_truth[0].header.stamp)
        if elapsed < self.settle_window:
            self.ground_truth.append(message)

    def publish_command(self, command, duration, wall_deadline):
        """Publish a Twist until simulation advances by duration, then stop."""
        stop = Twist()
        start = self.latest_ground_truth
        elapsed = 0.0
        while (
            rclpy.ok()
            and time.monotonic() < wall_deadline
            and elapsed < duration
        ):
            self.command_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
            if start is not None and self.latest_ground_truth is not None:
                elapsed = (
                    stamp_seconds(self.latest_ground_truth.header.stamp)
                    - stamp_seconds(start.header.stamp)
                )
        for _ in range(5):
            self.command_publisher.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)
        return elapsed

    def validate_motion(self, start, end, command_elapsed):
        """Return errors if a short forward command did not move mostly forward."""
        if start is None or end is None:
            return ["no ground-truth sample available to validate movement"]
        if not math.isfinite(command_elapsed):
            return [f"{GROUND_TRUTH_TOPIC}: command-window duration is non-finite"]
        if command_elapsed < self.command_duration:
            return [
                f"{GROUND_TRUTH_TOPIC}: simulation advanced only "
                f"{command_elapsed:.2f}s of the {self.command_duration:.1f}s "
                "forward /cmd_vel command before the wall timeout -- cannot "
                "validate movement"
            ]
        start_position = start.pose.pose.position
        end_position = end.pose.pose.position
        start_orientation = start.pose.pose.orientation
        pose_values = (
            start_position.x, start_position.y,
            end_position.x, end_position.y,
            start_orientation.x, start_orientation.y,
            start_orientation.z, start_orientation.w,
        )
        if not all(math.isfinite(value) for value in pose_values):
            return [f"{GROUND_TRUTH_TOPIC}: movement pose has non-finite values"]

        delta_x = end_position.x - start_position.x
        delta_y = end_position.y - start_position.y
        initial_yaw = quaternion_to_yaw(start_orientation)
        forward_displacement = (
            math.cos(initial_yaw) * delta_x + math.sin(initial_yaw) * delta_y
        )
        lateral_displacement = (
            -math.sin(initial_yaw) * delta_x + math.cos(initial_yaw) * delta_y
        )

        errors = []
        if forward_displacement < 0.0:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: moved {abs(forward_displacement):.3f}m "
                f"backward over a {self.command_duration:.1f}s forward /cmd_vel "
                f"command (expected at least {self.meaningful_motion:.3f}m "
                "forward)"
            )
        elif forward_displacement < self.meaningful_motion:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: moved only "
                f"{forward_displacement:.3f}m forward over a "
                f"{self.command_duration:.1f}s forward /cmd_vel command "
                f"(expected at least {self.meaningful_motion:.3f}m) -- wheels "
                "may be spinning without translating"
            )
        if abs(lateral_displacement) > MAX_LATERAL_DISPLACEMENT_M:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: moved {abs(lateral_displacement):.3f}m "
                f"laterally over a forward /cmd_vel command (maximum allowed "
                f"{MAX_LATERAL_DISPLACEMENT_M:.3f}m) -- motion was not "
                "predominantly forward"
            )
        return errors

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
        elapsed = stamp_seconds(last.header.stamp) - stamp_seconds(first.header.stamp)
        if elapsed < self.settle_window:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: only observed {elapsed:.2f}s of the "
                f"{self.settle_window:.1f}s settle window before timing out "
                "-- cannot confirm the robot settled")

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

        spawn_offset = math.hypot(first.pose.pose.position.x, first.pose.pose.position.y)
        if spawn_offset > MAX_SPAWN_OFFSET_M:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: settled {spawn_offset:.3f}m from the "
                f"expected (0, 0) spawn point -- the robot was likely pushed "
                "off spawn before the probe started observing")

        first_position = first.pose.pose.position
        observed_positions = []
        for index, message in enumerate(self.ground_truth):
            position = message.pose.pose.position
            values = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in values):
                errors.append(
                    f"{GROUND_TRUTH_TOPIC}: sample {index} position has "
                    "non-finite values"
                )
                continue
            observed_positions.append(values)
        if errors:
            return errors

        max_excursion = max(
            math.dist(
                (first_position.x, first_position.y, first_position.z),
                position,
            )
            for position in observed_positions
        )
        if max_excursion > MAX_SETTLED_EXCURSION_M:
            errors.append(
                f"{GROUND_TRUTH_TOPIC}: reached a maximum excursion of "
                f"{max_excursion:.3f}m from its first observed pose with no "
                f"command over {self.settle_window:.1f}s -- likely an initial "
                "collision or spawn on unstable geometry")

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

        pre_move = node.latest_ground_truth
        move = Twist()
        move.linear.x = node.command_speed
        command_elapsed = node.publish_command(
            move, node.command_duration, deadline)
        motion_errors = node.validate_motion(
            pre_move, node.latest_ground_truth, command_elapsed)
        if motion_errors:
            node.get_logger().error(
                "World smoke contract FAILED: " + "; ".join(motion_errors))
            return 1

        node.get_logger().info("World smoke contract PASSED: " + node.summary())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
