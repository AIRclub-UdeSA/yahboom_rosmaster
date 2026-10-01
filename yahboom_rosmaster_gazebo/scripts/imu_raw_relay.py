#!/usr/bin/env python3
"""
Publish /imu/data_raw with exactly the content the physical driver puts there.

physical_rosmaster's X3 driver (yahboomcar_bringup/Mcnamu_driver_X3.py at
468662c, ``publish_data``) builds ``Imu()`` and fills in only the header and the
acceleration and angular velocity. Everything else keeps the message default: the
identity quaternion as orientation, and all three covariance matrices zero. It
publishes with the default QoS, Reliable with a depth of 100. Gazebo's IMU
reports its own world orientation and whatever covariance the bridge derives, so
this node copies the three fields the robot fills and leaves the rest as the
robot leaves it. imu_filter_madgwick then estimates the orientation from it, as
on the robot (#43 step 8c).

The node does no arithmetic and holds no clock: it only reacts to a message.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# The driver's own publisher: ``create_publisher(Imu, "/imu/data_raw", 100)``,
# which is Reliable and keeps the last 100.
PUBLISH_DEPTH = 100


class ImuRawRelay(Node):
    """Republish bridged IMU measurements as the robot's raw IMU message."""

    def __init__(self):
        super().__init__("imu_raw_relay")
        self.declare_parameter("input_topic", "/internal/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data_raw")
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(Imu, output_topic, PUBLISH_DEPTH)
        self.create_subscription(Imu, input_topic, self.republish, 10)
        self.get_logger().info(
            f"Relaying {input_topic} to {output_topic} with the physical driver's content")

    def republish(self, message):
        raw = Imu()
        raw.header = message.header
        raw.linear_acceleration = message.linear_acceleration
        raw.angular_velocity = message.angular_velocity
        self.publisher.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRawRelay()
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
