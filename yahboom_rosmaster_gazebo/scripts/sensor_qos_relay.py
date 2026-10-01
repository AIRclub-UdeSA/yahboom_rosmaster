#!/usr/bin/env python3
"""
Republish one bridged sensor topic from Reliable to Best Effort.

Neither ros_gz_bridge's parameter_bridge nor ros_gz_image's image_bridge in
this release expose a per-topic QoS override, so every Gazebo-bridged topic
publishes at the ROS 2 default (Reliable). The physical ROSMASTER X3
publishes every raw sensor stream -- LiDAR and both RGB-D image/camera_info
pairs -- as Best Effort (``qos_profile_sensor_data``); a consumer built for
that contract silently receives nothing from a Reliable topic. This node
relays one bridged /internal/... topic to its public contract name at Best
Effort so the same consumer works against sim and hardware without remaps.

For the LiDAR it also applies the sensor profile (config/sensor_profiles.yaml):
under ``physical`` it holds each scan back until the next arrives, which is how
late the robot's scan is, and it sets ``scan_time`` to the robot's one sweep.
"""

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class HoldBack:
    """
    Release each message when ``count`` later ones have arrived.

    With a count of one, a message leaves when the next arrives: one message
    period after it was produced, with its own header stamp untouched. That is
    how the physical LiDAR's scan reaches its consumers (the stamp marks the
    start of the revolution and the message leaves when it completes), and
    counting messages needs no clock, so no subscription to /clock (#43 step 8b).
    A count of zero releases every message at once.

    Three edges follow from it. The first message waits for the second. The last
    message before the stream ends is never released. If one message never
    arrives, its predecessor waits two periods.
    """

    def __init__(self, count):
        """Hold back ``count`` messages, which must not be negative."""
        if count < 0:
            raise ValueError(f"cannot hold back {count} messages")
        self.count = count
        self.held = deque()

    def push(self, message):
        """Take in a message; return the one to publish now, or None."""
        self.held.append(message)
        if len(self.held) > self.count:
            return self.held.popleft()
        return None


MESSAGE_TYPES = {
    "Image": Image,
    "CameraInfo": CameraInfo,
    "LaserScan": LaserScan,
}


class SensorQosRelay(Node):
    """Bridge-side QoS fix: Reliable /internal/... topic -> Best Effort public topic."""

    def __init__(self):
        super().__init__("sensor_qos_relay")
        self.declare_parameter("msg_type", "")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        # How many messages to hold back, from the sensor profile (LiDAR only).
        self.declare_parameter("hold_scans", 0)
        # The LaserScan scan_time to publish; negative keeps the message's own.
        # The bridge leaves it at zero, the robot's sllidar driver sets one sweep.
        self.declare_parameter("scan_time", -1.0)

        msg_type_name = self.get_parameter("msg_type").value
        if msg_type_name not in MESSAGE_TYPES:
            raise ValueError(
                f"Unsupported msg_type {msg_type_name!r}; expected one of "
                f"{sorted(MESSAGE_TYPES)}")
        message_type = MESSAGE_TYPES[msg_type_name]

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        if not self.input_topic or not self.output_topic:
            raise ValueError("input_topic and output_topic are required")

        self.hold_back = HoldBack(int(self.get_parameter("hold_scans").value))
        self.scan_time = float(self.get_parameter("scan_time").value)
        if self.scan_time >= 0.0 and message_type is not LaserScan:
            raise ValueError("scan_time only applies to LaserScan")

        self.publisher = self.create_publisher(
            message_type, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            message_type, self.input_topic, self.republish, 10)
        held = (
            f", each message {self.hold_back.count} later" if self.hold_back.count
            else "")
        self.get_logger().info(
            f"Relaying {self.input_topic} to {self.output_topic} at Best Effort "
            f"QoS{held}")

    def republish(self, message):
        message = self.hold_back.push(message)
        if message is None:
            return
        if self.scan_time >= 0.0:
            message.scan_time = self.scan_time
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SensorQosRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
