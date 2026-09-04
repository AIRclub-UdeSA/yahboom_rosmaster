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
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

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

        self.publisher = self.create_publisher(
            message_type, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            message_type, self.input_topic, self.republish, 10)
        self.get_logger().info(
            f"Relaying {self.input_topic} to {self.output_topic} at Best Effort QoS")

    def republish(self, message):
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
