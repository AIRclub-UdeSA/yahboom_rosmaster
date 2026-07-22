#!/usr/bin/env python3
"""Static contract tests for selectable mecanum wheel-contact profiles."""

import ast
import math
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


GAZEBO_PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = GAZEBO_PACKAGE.parent
PROFILE_CONFIG = GAZEBO_PACKAGE / "config" / "motion_profiles.yaml"
LAUNCH_FILE = (
    GAZEBO_PACKAGE / "launch" / "rosmaster_gazebo_fortress.launch.py"
)
ROBOT_XACRO = (
    REPOSITORY_ROOT
    / "yahboom_rosmaster_description"
    / "urdf"
    / "robots"
    / "rosmaster_x3.urdf.xacro"
)
PROFILE_KEYS = {
    "wheel_mu",
    "wheel_mu2",
    "wheel_slip2",
    "front_left_slip1",
    "front_right_slip1",
    "back_left_slip1",
    "back_right_slip1",
}
WHEEL_SIDES = ("front_left", "front_right", "back_left", "back_right")


def load_profiles():
    """Load the source profile document used by the launch file."""
    with PROFILE_CONFIG.open(encoding="utf-8") as profile_file:
        document = yaml.safe_load(profile_file)
    return document["profiles"]


def expand_profile(profile):
    """Expand the robot xacro with one complete wheel-contact profile."""
    command = [
        "xacro",
        str(ROBOT_XACRO),
        "use_gazebo:=true",
        "robot_name:=rosmaster_x3",
        "prefix:=",
    ]
    command.extend(f"{key}:={profile[key]}" for key in sorted(PROFILE_KEYS))
    result = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


class TestMotionProfileContract(unittest.TestCase):
    """Require profiles to select explicit, correctly expanded contact values."""

    @classmethod
    def setUpClass(cls):
        cls.profiles = load_profiles()

    def test_launch_defaults_to_stress_and_limits_choices(self):
        tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
        declarations = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "DeclareLaunchArgument" or not node.args:
                continue
            if isinstance(node.args[0], ast.Constant):
                declarations.append(node)

        motion_profile_declarations = [
            declaration
            for declaration in declarations
            if declaration.args[0].value == "motion_profile"
        ]
        self.assertEqual(len(motion_profile_declarations), 1)
        keywords = {
            keyword.arg: keyword.value
            for keyword in motion_profile_declarations[0].keywords
        }
        self.assertEqual(ast.literal_eval(keywords["default_value"]), "stress")
        self.assertEqual(
            set(ast.literal_eval(keywords["choices"])), {"ideal", "stress"})

    def test_ideal_is_exact_zero_slip_baseline(self):
        ideal = self.profiles["ideal"]
        self.assertEqual(set(ideal), PROFILE_KEYS)
        self.assertEqual(float(ideal["wheel_mu"]), 1.0)
        self.assertEqual(float(ideal["wheel_mu2"]), 0.0)
        self.assertEqual(float(ideal["wheel_slip2"]), 0.0)
        for side in WHEEL_SIDES:
            self.assertEqual(float(ideal[f"{side}_slip1"]), 0.0)

    def test_stress_is_nonideal_asymmetric_and_finite(self):
        ideal = self.profiles["ideal"]
        stress = self.profiles["stress"]
        self.assertEqual(set(stress), PROFILE_KEYS)
        for key, value in stress.items():
            self.assertTrue(math.isfinite(float(value)), key)
            self.assertGreaterEqual(float(value), 0.0, key)
        self.assertNotEqual(stress, ideal)
        self.assertLess(float(stress["wheel_mu"]), float(ideal["wheel_mu"]))
        self.assertGreater(float(stress["wheel_mu2"]), float(ideal["wheel_mu2"]))
        self.assertGreater(
            float(stress["wheel_slip2"]), float(ideal["wheel_slip2"]))
        slip1_values = {
            float(stress[f"{side}_slip1"])
            for side in WHEEL_SIDES
        }
        self.assertGreater(len(slip1_values), 1)
        self.assertTrue(all(value > 0.0 for value in slip1_values))

    def test_profiles_expand_into_each_wheel_and_one_ground_truth_plugin(self):
        for profile_name in ("ideal", "stress"):
            with self.subTest(profile=profile_name):
                profile = self.profiles[profile_name]
                robot = expand_profile(profile)

                for side in WHEEL_SIDES:
                    gazebo = robot.find(
                        f"./gazebo[@reference='{side}_wheel_link']")
                    self.assertIsNotNone(gazebo, side)
                    friction = gazebo.find("./collision/surface/friction/ode")
                    self.assertIsNotNone(friction, side)
                    expected = {
                        "mu": profile["wheel_mu"],
                        "mu2": profile["wheel_mu2"],
                        "slip1": profile[f"{side}_slip1"],
                        "slip2": profile["wheel_slip2"],
                    }
                    for element_name, expected_value in expected.items():
                        element = friction.find(element_name)
                        self.assertIsNotNone(element, f"{side}/{element_name}")
                        self.assertAlmostEqual(
                            float(element.text), float(expected_value), places=12)

                odometry_publishers = [
                    plugin
                    for plugin in robot.findall("./gazebo/plugin")
                    if plugin.get("name")
                    == "ignition::gazebo::systems::OdometryPublisher"
                ]
                self.assertEqual(len(odometry_publishers), 1)
                publisher = odometry_publishers[0]
                self.assertEqual(publisher.findtext("odom_frame"), "world")
                self.assertEqual(
                    publisher.findtext("robot_base_frame"), "base_footprint")
                self.assertEqual(
                    publisher.findtext("odom_topic"),
                    "/model/rosmaster_x3/ground_truth",
                )


if __name__ == "__main__":
    unittest.main()
