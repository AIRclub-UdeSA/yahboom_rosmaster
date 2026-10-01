#!/usr/bin/env python3
"""Tie the Madgwick filter's parameters to the parity ledger and the launch."""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ElementTree

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from real_robot_contract import RealRobotContract  # noqa: E402

CONFIG = PACKAGE_DIR / "config" / "imu_filter_madgwick.yaml"
LAUNCH = PACKAGE_DIR / "launch" / "rosmaster_gazebo_fortress.launch.py"
LEDGER = RealRobotContract.load()


def parameters():
    """Return the filter's parameters as the config file gives them to the node."""
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document["imu_filter_madgwick"]["ros__parameters"]


class TestImuFilterConfig(unittest.TestCase):
    """The parameters are the robot's, all of them, and only those."""

    def test_the_parameters_are_exactly_the_ledgers(self):
        self.assertEqual(parameters(), LEDGER.physical("imu.madgwick"))

    def test_the_config_is_keyed_by_the_node_name(self):
        # imu_filter_madgwick 2.1.5 names its node imu_filter_madgwick, which is
        # also what the ledger's orientation source says is publishing.
        document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(list(document), [LEDGER.nominal("imu.orientation_source")])

    def test_the_covariance_is_the_stddev_squared(self):
        stddev = parameters()["orientation_stddev"]
        for variance in LEDGER.nominal("imu.orientation_covariance_diag"):
            self.assertAlmostEqual(variance, stddev ** 2, places=12)

    def test_the_filter_does_not_publish_tf_or_use_the_magnetometer(self):
        self.assertFalse(parameters()["publish_tf"])
        self.assertFalse(parameters()["use_mag"])

    def test_the_launch_passes_this_file_to_the_stock_node(self):
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn('package="imu_filter_madgwick"', text)
        self.assertIn('executable="imu_filter_madgwick_node"', text)
        self.assertIn('"imu_filter_madgwick.yaml"', text)

    def test_the_package_depends_on_the_filter(self):
        package = ElementTree.parse(PACKAGE_DIR / "package.xml").getroot()
        depends = {
            element.text for element in package
            if element.tag in ("depend", "exec_depend")}
        self.assertIn("imu_filter_madgwick", depends)


if __name__ == "__main__":
    unittest.main()
