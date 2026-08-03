#!/usr/bin/env python3
"""Publish ideal odometry by integrating raw cmd_vel commands with a timeout."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


POSE_COVARIANCE_DIAGONAL = (0.001, 0.001, 0.001, 0.001, 0.001, 0.01)
TWIST_COVARIANCE_DIAGONAL = (0.001, 0.001, 0.001, 0.001, 0.001, 0.01)


def diagonal_covariance(values):
    covariance = [0.0] * 36
    for index, value in enumerate(values):
        covariance[index * 6 + index] = value
    return covariance


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CmdVelOdometry(Node):
    def __init__(self):
        super().__init__("cmd_vel_odometry")

        self.declare_parameter("odom_frame_id", "calc_odom")
        self.declare_parameter("base_frame_id", "calc_base")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("timeout", 0.5)

        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        publish_rate = self.get_parameter("publish_rate").value
        self.timeout = self.get_parameter("timeout").value

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # Velocity state
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_yaw = 0.0

        self.pose_covariance = diagonal_covariance(POSE_COVARIANCE_DIAGONAL)
        self.twist_covariance = diagonal_covariance(TWIST_COVARIANCE_DIAGONAL)

        self.odom_publisher = self.create_publisher(Odometry, "/calc_odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to pure /cmd_vel before errors are injected
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.last_time
        
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Publishing pure cmd_vel odometry to /{self.odom_frame_id} with {self.timeout}s watchdog")

    def cmd_vel_callback(self, msg: Twist):
        """Update current velocities and reset the watchdog timer."""
        self.v_x = msg.linear.x
        self.v_y = msg.linear.y
        self.v_yaw = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def timer_callback(self):
        """Integrate velocities over dt and publish."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Watchdog logic: force zero velocity if timeout exceeded
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds * 1e-9
        if time_since_last_cmd > self.timeout:
            self.v_x = 0.0
            self.v_y = 0.0
            self.v_yaw = 0.0

        if dt <= 0.0:
            return

        # Midpoint integration
        midpoint_heading = self.heading + (self.v_yaw * dt * 0.5)
        
        self.x += (
            self.v_x * math.cos(midpoint_heading) - 
            self.v_y * math.sin(midpoint_heading)
        ) * dt
        
        self.y += (
            self.v_x * math.sin(midpoint_heading) + 
            self.v_y * math.cos(midpoint_heading)
        ) * dt
        
        self.heading = normalize_angle(self.heading + self.v_yaw * dt)

        stamp = current_time.to_msg()
        self.publish_odometry(stamp, self.v_x, self.v_y, self.v_yaw)

    def publish_odometry(self, stamp, linear_x, linear_y, angular_z):
        qx, qy, qz, qw = quaternion_from_yaw(self.heading)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self.pose_covariance
        
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z
        odom.twist.covariance = self.twist_covariance
        
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = CmdVelOdometry()
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