#!/usr/bin/env python3
"""
Broadcast the simulator's ground-truth pose as a TF frame.

/ground_truth/odom is measurement-only and deliberately outside the robot's own
TF tree, so nothing can accidentally navigate on a perfect pose. Publishing it
under a separate `ground_truth_base` frame keeps that separation while letting
RViz show the true pose next to the estimated one, which is what makes odometry
drift visible.

This is a runtime node. The matching contract check lives in
scripts/ground_truth_contract_probe.py, which must stay a probe: it validates
and exits, whereas this runs for the life of the simulation.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GroundTruthTf(Node):
    """Republish ground-truth odometry as a transform."""

    def __init__(self):
        super().__init__("ground_truth_tf")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "ground_truth_base")

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        input_topic = self.get_parameter("input_topic").value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, input_topic, self.odom_callback, 10)

        self.get_logger().info(
            f"Broadcasting {self.frame_id} -> {self.child_frame_id} from {input_topic}")

    def odom_callback(self, message):
        """Convert one ground-truth odometry message into a transform."""
        transform = TransformStamped()
        # Reuse the simulator's stamp so the transform stays on sim time.
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthTf()
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
