#!/usr/bin/env python3
"""
Republish the Fortress RGB-D point cloud under its true frame id.

Fortress 6.18 stamps the rgbd_camera's PointCloudPacked with the sensor's
``optical_frame_id`` even though the XYZ data is expressed in the camera's
regular +X-forward frame. Relabeling the header here (instead of collapsing
the optical TF rotation) keeps image and calibration topics on the REP-104
optical frame while giving the cloud the ``cam_1_depth_frame`` label its
coordinates actually use, so RViz, TF consumers, and depth pipelines all
agree.

The public output also switches to Best Effort here: the physical robot
publishes its point cloud as Best Effort (``qos_profile_sensor_data``), and
ros_gz_bridge cannot publish that directly, so this relay is also where the
QoS gets fixed to match the contract.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRelay(Node):
    """Bridge-side relabel: gz /cam_1/points -> public RGB-D cloud topic."""

    def __init__(self):
        super().__init__("pointcloud_frame_relay")
        self.declare_parameter("input_topic", "/internal/cam_1/points_raw")
        self.declare_parameter("output_topic", "cam_1/depth/color/points")
        self.declare_parameter("target_frame", "cam_1_depth_frame")
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value

        qos_in = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic,
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.input_topic,
                                 self.republish, qos_in)
        self.get_logger().info(
            f"Relabeling {self.input_topic} frame to "
            f"{self.target_frame!r} on {self.output_topic}")

    def republish(self, message):
        message.header.frame_id = self.target_frame
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameRelay()
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
