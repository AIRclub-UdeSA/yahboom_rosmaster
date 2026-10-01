#!/usr/bin/env python3
"""Check wheel_state_odometry's covariance and its config against the ledger."""

from pathlib import Path
import sys
import unittest

import rclpy
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from real_robot_contract import RealRobotContract  # noqa: E402
from wheel_state_odometry import (  # noqa: E402
    COVARIANCE_PARAMETERS,
    WheelStateOdometry,
    planar_covariance,
)

CONFIG = PACKAGE_DIR / "config" / "wheel_odometry.yaml"


def configured():
    """Return the parameters config/wheel_odometry.yaml gives the node."""
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return data["wheel_state_odometry"]["ros__parameters"]


class TestConfig(unittest.TestCase):
    """The file the launch passes to the node holds the robot's covariance."""

    def test_values_equal_the_ledger(self):
        ledger = RealRobotContract.load()
        parameters = configured()
        self.assertEqual(set(parameters), set(COVARIANCE_PARAMETERS))
        for kind in ("pose", "twist"):
            key = f"odometry.{kind}_covariance_x_y_yaw"
            values = [parameters[f"{kind}_covariance_{axis}"] for axis in ("x", "y", "yaw")]
            with self.subTest(key=key):
                self.assertEqual(values, ledger.physical(key))
                self.assertEqual(values, ledger.nominal(key))


class TestNode(unittest.TestCase):
    """The covariance on /odom is the configured one, in the robot's layout."""

    def setUp(self):
        arguments = ["--ros-args"]
        for name, value in configured().items():
            arguments += ["-p", f"{name}:={value}"]
        rclpy.init(args=arguments)
        self.node = WheelStateOdometry()
        self.published = []
        self.node.odom_publisher.publish = self.published.append

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_only_x_y_and_yaw_are_filled(self):
        self.node.publish_odometry(self.node.get_clock().now().to_msg(), 0.0, 0.0, 0.0)
        message = self.published[-1]
        for covariance, expected in (
                (message.pose.covariance, (0.001, 0.001, 0.001)),
                (message.twist.covariance, (0.0001, 0.0001, 0.0001))):
            self.assertEqual(list(covariance), planar_covariance(*expected))
            self.assertEqual(
                [index for index, value in enumerate(covariance) if value != 0.0],
                [0, 7, 35])

    def test_missing_covariance_is_an_error(self):
        self.node.destroy_node()
        rclpy.shutdown()
        rclpy.init()
        with self.assertRaisesRegex(ValueError, "pose_covariance_x"):
            WheelStateOdometry()
        # tearDown shuts rclpy down again, so leave a live node and context.
        rclpy.shutdown()
        rclpy.init(args=["--ros-args", "-p", "pose_covariance_x:=0.0"])
        self.node = rclpy.create_node("placeholder")


if __name__ == "__main__":
    unittest.main()
