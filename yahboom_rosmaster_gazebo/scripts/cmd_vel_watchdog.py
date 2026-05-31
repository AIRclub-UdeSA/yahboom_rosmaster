#!/usr/bin/env python3
"""Watch /cmd_vel and feed native Gazebo MecanumDrive with a timeout."""

import copy
import time

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

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.last_cmd = Twist()
        self.last_cmd_time = None

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"Relaying {self.input_topic} to {self.output_topic} "
            f"with {self.timeout:.2f}s watchdog timeout")

    def cmd_vel_callback(self, msg):
        self.last_cmd = copy.deepcopy(msg)
        self.last_cmd_time = time.monotonic()

    def timer_callback(self):
        if self.last_cmd_time is None:
            self.publisher.publish(Twist())
            return

        if time.monotonic() - self.last_cmd_time > self.timeout:
            self.publisher.publish(Twist())
            return

        self.publisher.publish(self.last_cmd)


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
