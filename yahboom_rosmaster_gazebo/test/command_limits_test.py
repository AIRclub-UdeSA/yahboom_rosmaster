#!/usr/bin/env python3
"""Check the command limits: their file, the clamp, and the watchdog that applies it."""

from pathlib import Path
import math
import sys
import tempfile
import unittest

import rclpy
import yaml
from geometry_msgs.msg import Twist

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from cmd_vel_watchdog import CmdVelWatchdog  # noqa: E402
from command_limits import (  # noqa: E402
    LIMIT_KEYS,
    SOURCE_PATH,
    clamp_command,
    load_command_limits,
)
from real_robot_contract import RealRobotContract  # noqa: E402

LEDGER_KEYS = (
    "command.linear_x_limit_mps",
    "command.linear_y_limit_mps",
    "command.angular_z_limit_rad_s",
)
LIMITS = (1.0, 1.0, 5.0)


def edited_limits(edit):
    """Load the limits file after ``edit`` changed its parsed document."""
    data = yaml.safe_load(SOURCE_PATH.read_text(encoding="utf-8"))
    edit(data["limits"])
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "command_limits.yaml"
        path.write_text(yaml.safe_dump(data), encoding="utf-8")
        return load_command_limits(path)


class TestShippedLimits(unittest.TestCase):
    """The file the launch loads is the physical driver's, and says where from."""

    def test_limits_equal_the_ledger(self):
        # The ledger holds the physical figures and the simulator's claim about
        # them; both must agree with the file the watchdog is configured from.
        ledger = RealRobotContract.load()
        loaded = load_command_limits(SOURCE_PATH)
        for key, value in zip(LEDGER_KEYS, loaded):
            with self.subTest(key=key):
                self.assertEqual(ledger.physical(key), value)
                self.assertEqual(ledger.nominal(key), value)

    def test_every_value_names_its_source(self):
        data = yaml.safe_load(SOURCE_PATH.read_text(encoding="utf-8"))
        for key in LIMIT_KEYS:
            with self.subTest(key=key):
                self.assertIn("x3_driver.yaml", data["limits"][key]["source"])

    def test_rejects_a_missing_or_invalid_value(self):
        with self.assertRaisesRegex(RuntimeError, "exactly"):
            edited_limits(lambda limits: limits.pop("angular_z_rad_s"))
        with self.assertRaisesRegex(RuntimeError, "source"):
            edited_limits(lambda limits: limits["linear_x_mps"].pop("source"))
        with self.assertRaisesRegex(RuntimeError, "non-negative"):
            edited_limits(lambda limits: limits["linear_x_mps"].update(value=-1.0))
        with self.assertRaisesRegex(RuntimeError, "numeric"):
            edited_limits(lambda limits: limits["linear_y_mps"].update(value="fast"))


class TestClamp(unittest.TestCase):
    """The semantics of the physical driver's clamp_motion_command."""

    def test_inside_the_limits_is_unchanged(self):
        self.assertEqual(clamp_command(0.4, -0.7, 2.0, *LIMITS), (0.4, -0.7, 2.0))

    def test_each_component_saturates_on_its_own(self):
        # Saturation, not scaling: the vector keeps its shape only inside the
        # limits, and an over-limit x leaves y untouched.
        self.assertEqual(clamp_command(3.0, 0.5, 0.0, *LIMITS), (1.0, 0.5, 0.0))
        self.assertEqual(clamp_command(-3.0, 2.0, -9.0, *LIMITS), (-1.0, 1.0, -5.0))

    def test_a_non_finite_command_is_a_full_stop(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad):
                self.assertEqual(clamp_command(bad, 0.5, 0.5, *LIMITS), (0.0, 0.0, 0.0))
                self.assertEqual(clamp_command(0.5, 0.5, bad, *LIMITS), (0.0, 0.0, 0.0))

    def test_a_non_finite_limit_is_a_full_stop(self):
        self.assertEqual(
            clamp_command(0.5, 0.5, 0.5, math.inf, 1.0, 5.0), (0.0, 0.0, 0.0))


class TestWatchdog(unittest.TestCase):
    """The watchdog clamps first, then biases, and still refreshes its timeout."""

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[
            "--ros-args",
            "-p", "linear_x_limit:=1.0",
            "-p", "linear_y_limit:=1.0",
            "-p", "angular_z_limit:=5.0",
        ])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CmdVelWatchdog()

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def command(x=0.0, y=0.0, z=0.0):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = x, y, z
        return message

    def test_an_over_limit_command_is_clamped(self):
        self.node.cmd_vel_callback(self.command(3.0, -2.0, 9.0))
        last = self.node.last_cmd
        self.assertEqual((last.linear.x, last.linear.y, last.angular.z), (1.0, -1.0, 5.0))

    def test_the_timeout_is_refreshed_by_a_clamped_command(self):
        self.assertIsNone(self.node.last_cmd_time)
        self.node.cmd_vel_callback(self.command(3.0))
        self.assertIsNotNone(self.node.last_cmd_time)

    def test_the_bias_acts_on_the_clamped_command(self):
        # A forward bias of 0.2 m/s of y per m/s of x. Biasing the raw 3 m/s
        # would give 0.6; the drivetrain only ever receives the clamped 1.0.
        self.node.biases = {"forward": {"y": 0.2}}
        self.node.cmd_vel_callback(self.command(3.0))
        last = self.node.last_cmd
        self.assertEqual(last.linear.x, 1.0)
        self.assertAlmostEqual(last.linear.y, 0.2)

    def test_a_non_finite_command_becomes_zero(self):
        self.node.cmd_vel_callback(self.command(math.nan, 0.5, 0.5))
        last = self.node.last_cmd
        self.assertEqual((last.linear.x, last.linear.y, last.angular.z), (0.0, 0.0, 0.0))

    def test_missing_limits_are_an_error(self):
        rclpy.shutdown()
        rclpy.init()
        try:
            with self.assertRaisesRegex(ValueError, "linear_x_limit"):
                CmdVelWatchdog()
        finally:
            rclpy.shutdown()
            rclpy.init(args=[
                "--ros-args",
                "-p", "linear_x_limit:=1.0",
                "-p", "linear_y_limit:=1.0",
                "-p", "angular_z_limit:=5.0",
            ])


if __name__ == "__main__":
    unittest.main()
