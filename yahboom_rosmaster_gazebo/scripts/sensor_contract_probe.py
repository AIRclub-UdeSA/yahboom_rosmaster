#!/usr/bin/env python3
"""Validate the standalone simulator's public sensor and state contract."""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


EXPECTED_WHEEL_JOINTS = {
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
}


class SensorContractProbe(Node):
    """Collect consecutive messages and validate their functional contract."""

    def __init__(self):
        super().__init__("sensor_contract_probe")
        self.declare_parameter("timeout", 35.0)
        self.declare_parameter("samples", 3)
        self.timeout = float(self.get_parameter("timeout").value)
        self.samples = max(2, int(self.get_parameter("samples").value))

        self.required_counts = {
            "/clock": self.samples,
            "/scan": self.samples,
            "/imu/data": self.samples,
            "/cam_1/color/image_raw": self.samples,
            "/cam_1/depth/image_raw": self.samples,
            "/cam_1/color/camera_info": self.samples,
            "/cam_1/depth/camera_info": self.samples,
            "/cam_1/depth/color/points": self.samples,
            "/joint_states": self.samples,
            "/odom": self.samples,
            "/tf": self.samples,
            "/tf_static": 1,
        }
        self.messages = {topic: [] for topic in self.required_counts}
        self._subscription_handles = []

        default_qos = QoSProfile(depth=20)
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        topic_types = {
            "/clock": Clock,
            "/scan": LaserScan,
            "/imu/data": Imu,
            "/cam_1/color/image_raw": Image,
            "/cam_1/depth/image_raw": Image,
            "/cam_1/color/camera_info": CameraInfo,
            "/cam_1/depth/camera_info": CameraInfo,
            "/cam_1/depth/color/points": PointCloud2,
            "/joint_states": JointState,
            "/odom": Odometry,
            "/tf": TFMessage,
            "/tf_static": TFMessage,
        }
        sensor_topics = {
            "/clock",
            "/scan",
            "/imu/data",
            "/cam_1/color/image_raw",
            "/cam_1/depth/image_raw",
            "/cam_1/color/camera_info",
            "/cam_1/depth/camera_info",
            "/cam_1/depth/color/points",
        }

        for topic, message_type in topic_types.items():
            if topic == "/tf_static":
                qos = static_tf_qos
            elif topic in sensor_topics:
                qos = sensor_qos
            else:
                qos = default_qos
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda message, topic_name=topic: self.capture(topic_name, message),
                qos,
            )
            self._subscription_handles.append(subscription)

        self.get_logger().info(
            f"Waiting up to {self.timeout:.1f}s for the standalone sensor contract")

    def capture(self, topic, message):
        """Keep only the number of messages needed by the contract."""
        if len(self.messages[topic]) < self.required_counts[topic]:
            self.messages[topic].append(message)

    def complete(self):
        """Return whether all topics have produced the required samples."""
        return all(
            len(self.messages[topic]) >= count
            for topic, count in self.required_counts.items()
        )

    @staticmethod
    def stamp_seconds(message):
        """Extract a header timestamp as seconds."""
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def finite(values):
        """Return whether every supplied numeric value is finite."""
        return all(math.isfinite(float(value)) for value in values)

    def validate_header(self, topic, errors):
        """Validate frame IDs and monotonically increasing timestamps."""
        messages = self.messages[topic]
        stamps = [self.stamp_seconds(message) for message in messages]
        if any(stamp <= 0.0 for stamp in stamps):
            errors.append(f"{topic}: header timestamp is zero or negative")
        if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
            errors.append(f"{topic}: header timestamps are not strictly increasing")
        if any(not message.header.frame_id for message in messages):
            errors.append(f"{topic}: frame_id is empty")

    def validate(self):
        """Return contract errors after collection finishes."""
        errors = []
        for topic, required in self.required_counts.items():
            received = len(self.messages[topic])
            if received < required:
                errors.append(f"{topic}: received {received}/{required} messages")
        if errors:
            return errors

        clock_times = [
            float(message.clock.sec) + float(message.clock.nanosec) * 1e-9
            for message in self.messages["/clock"]
        ]
        if clock_times[0] <= 0.0 or any(
                current <= previous
                for previous, current in zip(clock_times, clock_times[1:])):
            errors.append("/clock: simulation time is not positive and increasing")

        header_topics = (
            "/scan",
            "/imu/data",
            "/cam_1/color/image_raw",
            "/cam_1/depth/image_raw",
            "/cam_1/color/camera_info",
            "/cam_1/depth/camera_info",
            "/cam_1/depth/color/points",
            "/joint_states",
            "/odom",
        )
        for topic in header_topics:
            self.validate_header(topic, errors)

        color = self.messages["/cam_1/color/image_raw"][-1]
        depth = self.messages["/cam_1/depth/image_raw"][-1]
        if color.encoding != "rgb8":
            errors.append(f"color image: expected rgb8, got {color.encoding}")
        if depth.encoding != "32FC1":
            errors.append(f"depth image: expected 32FC1, got {depth.encoding}")
        for label, image in (("color", color), ("depth", depth)):
            if image.width == 0 or image.height == 0:
                errors.append(f"{label} image: dimensions are zero")
            if len(image.data) != image.step * image.height:
                errors.append(f"{label} image: data length does not match step*height")

        color_info = self.messages["/cam_1/color/camera_info"][-1]
        depth_info = self.messages["/cam_1/depth/camera_info"][-1]
        for label, info, image in (
                ("color", color_info, color), ("depth", depth_info, depth)):
            if (info.width, info.height) != (image.width, image.height):
                errors.append(f"{label} camera info: dimensions do not match image")
            if len(info.k) != 9 or not self.finite(info.k):
                errors.append(f"{label} camera info: invalid intrinsic matrix")
            if info.header.frame_id != image.header.frame_id:
                errors.append(f"{label} camera info: frame does not match image")

        points = self.messages["/cam_1/depth/color/points"][-1]
        point_fields = {field.name for field in points.fields}
        if not {"x", "y", "z", "rgb"}.issubset(point_fields):
            errors.append(f"point cloud: missing XYZRGB fields: {point_fields}")
        if points.width == 0 or points.height == 0 or not points.data:
            errors.append("point cloud: dimensions or data are empty")
        if len(points.data) != points.row_step * points.height:
            errors.append("point cloud: data length does not match row_step*height")

        scan = self.messages["/scan"][-1]
        if scan.header.frame_id != "laser_frame":
            errors.append(f"scan: expected laser_frame, got {scan.header.frame_id}")
        if not scan.ranges or scan.angle_increment <= 0.0:
            errors.append("scan: ranges are empty or angle increment is invalid")
        if not 0.0 < scan.range_min < scan.range_max:
            errors.append("scan: range bounds are invalid")
        if not self.finite((
                scan.angle_min, scan.angle_max, scan.angle_increment,
                scan.range_min, scan.range_max)):
            errors.append("scan: metadata contains non-finite values")

        imu = self.messages["/imu/data"][-1]
        if imu.header.frame_id != "imu_link":
            errors.append(f"imu: expected imu_link, got {imu.header.frame_id}")
        imu_values = (
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
            imu.linear_acceleration.x, imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        )
        if not self.finite(imu_values):
            errors.append("imu: motion values contain non-finite values")
        gravity = math.sqrt(
            imu.linear_acceleration.x ** 2 +
            imu.linear_acceleration.y ** 2 +
            imu.linear_acceleration.z ** 2)
        if not 5.0 < gravity < 15.0:
            errors.append(f"imu: stationary acceleration magnitude is {gravity:.3f}")

        joints = self.messages["/joint_states"][-1]
        if not EXPECTED_WHEEL_JOINTS.issubset(set(joints.name)):
            errors.append(f"joint states: missing wheel joints from {joints.name}")
        if len(joints.position) < len(joints.name):
            errors.append("joint states: missing position values")
        if len(joints.velocity) < len(joints.name):
            errors.append("joint states: missing velocity values")

        odometry = self.messages["/odom"][-1]
        if odometry.header.frame_id != "odom":
            errors.append(f"odom: expected frame odom, got {odometry.header.frame_id}")
        if odometry.child_frame_id != "base_footprint":
            errors.append(
                f"odom: expected child base_footprint, got {odometry.child_frame_id}")

        dynamic_children = {
            transform.child_frame_id
            for message in self.messages["/tf"]
            for transform in message.transforms
        }
        if "base_footprint" not in dynamic_children:
            errors.append("/tf: odom -> base_footprint transform was not observed")

        static_children = {
            transform.child_frame_id
            for message in self.messages["/tf_static"]
            for transform in message.transforms
        }
        required_static_frames = {
            "base_link", "laser_frame", "imu_link",
            "cam_1_depth_optical_frame", "cam_1_color_optical_frame",
        }
        missing_static = required_static_frames - static_children
        if missing_static:
            errors.append(f"/tf_static: missing frames {sorted(missing_static)}")

        return errors

    def summary(self):
        """Return a compact message-count summary."""
        return ", ".join(
            f"{topic}={len(messages)}"
            for topic, messages in self.messages.items()
        )


def main():
    rclpy.init()
    node = SensorContractProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
        errors = node.validate()
        if errors:
            node.get_logger().error("Sensor contract FAILED: " + "; ".join(errors))
            node.get_logger().error("Received: " + node.summary())
            return 1
        node.get_logger().info("Sensor contract PASSED: " + node.summary())
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
