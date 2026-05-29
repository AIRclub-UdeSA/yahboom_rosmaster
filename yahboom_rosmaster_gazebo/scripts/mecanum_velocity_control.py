#!/usr/bin/env python3
"""
Bridge /cmd_vel (body-frame Twist) → world-frame Twist for gz-sim VelocityControl plugin.

gz-sim VelocityControl applies velocity in the WORLD frame, but ROS nav2/teleop publishes
cmd_vel in the BODY frame (robot-local). This node reads the current robot yaw from
/mecanum_drive_controller/odom and transforms body-frame commands to world-frame before
publishing to /cmd_vel_world_frame (bridged to Gazebo's VelocityControl topic).

A watchdog timer publishes zeros if no /cmd_vel arrives within CMD_TIMEOUT seconds,
matching the mecanum_drive_controller's cmd_vel_timeout so the body stops when nav2 stops.

The mecanum_drive_controller still handles:
  - Wheel velocity commands (IK from body-frame cmd_vel → joint velocities)
  - Odometry computation and publishing
This node only handles body-frame → world-frame transform for VelocityControl.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_TIMEOUT = 0.5  # seconds — matches mecanum_drive_controller cmd_vel_timeout


class MecanumVelocityControl(Node):
    def __init__(self):
        super().__init__('mecanum_velocity_control')
        self._yaw = 0.0
        self._last_cmd_time = self.get_clock().now()

        self._sub_odom = self.create_subscription(
            Odometry, '/mecanum_drive_controller/odom', self._odom_cb, 10)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, 10)
        self._pub = self.create_publisher(Twist, '/cmd_vel_world_frame', 10)
        # Watchdog: publish zeros if /cmd_vel goes silent
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()
        vx = msg.linear.x
        vy = msg.linear.y
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)

        world = Twist()
        world.linear.x = vx * c - vy * s
        world.linear.y = vx * s + vy * c
        world.angular.z = msg.angular.z
        self._pub.publish(world)

    def _watchdog_cb(self) -> None:
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > CMD_TIMEOUT:
            self._pub.publish(Twist())  # all zeros → VelocityControl stops body


def main(args=None):
    rclpy.init(args=args)
    node = MecanumVelocityControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
