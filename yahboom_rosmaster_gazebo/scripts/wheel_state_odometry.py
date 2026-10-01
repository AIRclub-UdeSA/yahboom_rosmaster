#!/usr/bin/env python3
"""Publish mecanum odometry from wheel joint positions."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


COVARIANCE_PARAMETERS = (
    "pose_covariance_x", "pose_covariance_y", "pose_covariance_yaw",
    "twist_covariance_x", "twist_covariance_y", "twist_covariance_yaw",
)
# The row-major 6x6 indices of x, y and yaw: the only entries the physical
# robot's odometry.hpp fills.
COVARIANCE_INDICES = (0, 7, 35)


def planar_covariance(x, y, yaw):
    """Return a 6x6 covariance with only the x, y and yaw variances set."""
    covariance = [0.0] * 36
    for index, value in zip(COVARIANCE_INDICES, (x, y, yaw)):
        covariance[index] = value
    return covariance


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelStateOdometry(Node):
    def __init__(self):
        super().__init__("wheel_state_odometry")

        self.declare_parameter("front_left_joint", "front_left_wheel_joint")
        self.declare_parameter("front_right_joint", "front_right_wheel_joint")
        self.declare_parameter("back_left_joint", "back_left_wheel_joint")
        self.declare_parameter("back_right_joint", "back_right_wheel_joint")
        self.declare_parameter("wheel_base", 0.16)
        self.declare_parameter("wheel_separation", 0.169)
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("max_wheel_position_jump", 2.0 * math.pi)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        # config/wheel_odometry.yaml supplies these; they have no default.
        for name in COVARIANCE_PARAMETERS:
            self.declare_parameter(name, Parameter.Type.DOUBLE)

        self.front_left_joint = self.get_parameter("front_left_joint").value
        self.front_right_joint = self.get_parameter("front_right_joint").value
        self.back_left_joint = self.get_parameter("back_left_joint").value
        self.back_right_joint = self.get_parameter("back_right_joint").value
        self.joint_names = (
            self.front_left_joint,
            self.front_right_joint,
            self.back_left_joint,
            self.back_right_joint,
        )

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.wheel_separation = float(
            self.get_parameter("wheel_separation").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.max_wheel_position_jump = float(
            self.get_parameter("max_wheel_position_jump").value)
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.previous_positions = None
        self.previous_stamp = None

        try:
            covariance = [
                float(self.get_parameter(name).value)
                for name in COVARIANCE_PARAMETERS]
        except ParameterUninitializedException as error:
            raise ValueError(
                f"wheel_state_odometry is missing a required parameter "
                f"(config/wheel_odometry.yaml): {error}") from error
        self.pose_covariance = planar_covariance(*covariance[:3])
        self.twist_covariance = planar_covariance(*covariance[3:])

        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10)

        self.get_logger().info(
            "Publishing wheel-state odometry from /joint_states to /odom")

    def joint_state_callback(self, msg):
        positions = self.extract_positions(msg)
        if positions is None:
            return

        stamp = msg.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_positions is None:
            self.previous_positions = positions
            self.previous_stamp = stamp_seconds
            self.publish_odometry(stamp, 0.0, 0.0, 0.0)
            return

        if stamp_seconds < self.previous_stamp:
            self.get_logger().warn(
                "Joint-state time moved backwards; resetting integrated odometry")
            self.x = 0.0
            self.y = 0.0
            self.heading = 0.0
            self.previous_positions = positions
            self.previous_stamp = stamp_seconds
            self.publish_odometry(stamp, 0.0, 0.0, 0.0)
            return

        dt = stamp_seconds - self.previous_stamp
        if dt < 0.0001:
            return

        delta = {
            name: positions[name] - self.previous_positions[name]
            for name in self.joint_names
        }
        discontinuous = [
            name for name, value in delta.items()
            if abs(value) > self.max_wheel_position_jump
        ]
        if discontinuous:
            self.get_logger().warn(
                "Rebasing wheel-state odometry after a joint-position "
                f"discontinuity in {discontinuous}")
            self.previous_positions = positions
            self.previous_stamp = stamp_seconds
            self.publish_odometry(stamp, 0.0, 0.0, 0.0)
            return

        linear_x_delta = self.wheel_radius * (
            delta[self.front_left_joint] +
            delta[self.front_right_joint] +
            delta[self.back_left_joint] +
            delta[self.back_right_joint]) * 0.25

        linear_y_delta = self.wheel_radius * (
            -delta[self.front_left_joint] +
            delta[self.front_right_joint] +
            delta[self.back_left_joint] -
            delta[self.back_right_joint]) * 0.25

        angular_length = 0.5 * (self.wheel_base + self.wheel_separation)
        angular_delta = self.wheel_radius * (
            -delta[self.front_left_joint] +
            delta[self.front_right_joint] -
            delta[self.back_left_joint] +
            delta[self.back_right_joint]) / (4.0 * angular_length)

        midpoint_heading = self.heading + angular_delta * 0.5
        self.x += (
            linear_x_delta * math.cos(midpoint_heading) -
            linear_y_delta * math.sin(midpoint_heading))
        self.y += (
            linear_x_delta * math.sin(midpoint_heading) +
            linear_y_delta * math.cos(midpoint_heading))
        self.heading = normalize_angle(self.heading + angular_delta)

        self.previous_positions = positions
        self.previous_stamp = stamp_seconds

        self.publish_odometry(
            stamp,
            linear_x_delta / dt,
            linear_y_delta / dt,
            angular_delta / dt)

    def extract_positions(self, msg):
        if len(msg.position) < len(msg.name):
            self.get_logger().warn(
                "JointState message is missing position values",
                throttle_duration_sec=5.0)
            return None

        indexes = {name: index for index, name in enumerate(msg.name)}
        missing = [name for name in self.joint_names if name not in indexes]
        if missing:
            self.get_logger().warn(
                f"JointState message missing wheel joints: {missing}",
                throttle_duration_sec=5.0)
            return None

        positions = {
            name: msg.position[indexes[name]]
            for name in self.joint_names
        }
        if not all(math.isfinite(value) for value in positions.values()):
            return self.warn_nonfinite_positions()
        return positions

    def warn_nonfinite_positions(self):
        """Reject non-finite wheel positions without poisoning integration."""
        self.get_logger().warn(
            "JointState message contains non-finite wheel positions",
            throttle_duration_sec=5.0)
        return None

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
    node = WheelStateOdometry()
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
