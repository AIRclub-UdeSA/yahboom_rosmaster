#!/usr/bin/env python3
"""Watch /cmd_vel and feed native Gazebo MecanumDrive with a timeout and YAML-driven biases."""

import copy
import time
import yaml
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_gz")
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("publish_rate", 30.0)

        self.declare_parameter("motion_bias_file", "")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        bias_file = self.get_parameter("motion_bias_file").value

        # Load biases from YAML and randomly sample them once for this session
        self.biases = {}
        if bias_file:
            try:
                with open(bias_file, "r") as f:
                    doc = yaml.safe_load(f)
                    if doc and "motion_bias" in doc:
                        raw_biases = doc["motion_bias"]

                        # Sample static biases once per launch based on max values
                        for category, axes in raw_biases.items():
                            if category == "randomization":
                                self.biases[category] = axes
                                continue

                            self.biases[category] = {}
                            for axis, max_val in axes.items():
                                # Sample randomly between -max_val and max_val.
                                # This honors the user's specific +/- sign limit.
                                sampled_val = random.uniform(-float(max_val), float(max_val))
                                self.biases[category][axis] = sampled_val

                        self.get_logger().info(
                            f"Sampled randomized motion biases from {bias_file}")
                        self.get_logger().debug(f"Session Biases: {self.biases}")
            except Exception as e:
                self.get_logger().error(f"Failed to load motion bias file '{bias_file}': {e}")

        self.last_cmd = Twist()
        self.last_cmd_time = None

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Relaying {self.input_topic} to {self.output_topic} "
            f"with {self.timeout:.2f}s watchdog timeout")

    def _get_bias(self, category, axis, default=0.0):
        """Safely fetch a sampled value from the biases dict."""
        return self.biases.get(category, {}).get(axis, default)

    def cmd_vel_callback(self, msg):
        cmd = copy.deepcopy(msg)

        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z

        if self.biases:
            # 1. Apply sampled static biases based on the primary inputs
            # Forward
            if vx > 0:
                cmd.linear.y += self._get_bias("forward", "y") * vx
                cmd.angular.z += self._get_bias("forward", "omega") * vx
            # Backward
            elif vx < 0:
                cmd.linear.y += self._get_bias("backward", "y") * abs(vx)
                cmd.angular.z += self._get_bias("backward", "omega") * abs(vx)

            # Left strafe (standard ROS: +Y is left)
            if vy > 0:
                cmd.linear.x += self._get_bias("left", "x") * vy
                cmd.angular.z += self._get_bias("left", "omega") * vy
            # Right strafe (standard ROS: -Y is right)
            elif vy < 0:
                cmd.linear.x += self._get_bias("right", "x") * abs(vy)
                cmd.angular.z += self._get_bias("right", "omega") * abs(vy)

            # Rotate CCW (+Z)
            if w > 0:
                cmd.linear.x += self._get_bias("rotate_ccw", "x") * w
                cmd.linear.y += self._get_bias("rotate_ccw", "y") * w
            # Rotate CW (-Z)
            elif w < 0:
                cmd.linear.x += self._get_bias("rotate_cw", "x") * abs(w)
                cmd.linear.y += self._get_bias("rotate_cw", "y") * abs(w)

            # 2. Apply Randomization noise based on standard fraction
            std_fraction = self._get_bias("randomization", "std_fraction", 0.0)
            if std_fraction > 0:
                if vx != 0.0:
                    cmd.linear.x += random.gauss(0.0, abs(vx) * std_fraction)
                if vy != 0.0:
                    cmd.linear.y += random.gauss(0.0, abs(vy) * std_fraction)
                if w != 0.0:
                    cmd.angular.z += random.gauss(0.0, abs(w) * std_fraction)

        self.last_cmd = cmd
        self.last_cmd_time = time.monotonic()

    def stop_robot(self):
        """Send explicit zero velocities to bring base to a complete stop."""
        zero = Twist()
        for _ in range(3):
            try:
                self.publisher.publish(zero)
            except Exception:
                pass

    def timer_callback(self):
        if self.last_cmd_time is None:
            self.publisher.publish(Twist())
            return

        if time.monotonic() - self.last_cmd_time > self.timeout:
            self.publisher.publish(Twist())
            return

        self.publisher.publish(self.last_cmd)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelWatchdog()
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
