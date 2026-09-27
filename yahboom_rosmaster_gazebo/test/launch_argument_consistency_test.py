#!/usr/bin/env python3
"""
Cross-file consistency of the launch arguments both simulator entrypoints share.

sensor_profile, sensor_seed, cloud_strip_nan, cloud_decimation and
motion_profile are declared independently in rosmaster_x3_sim.launch.py (the
bringup entrypoint) and rosmaster_gazebo_fortress.launch.py (the file it
includes and passes them through to). A default, description or set of
choices that drifts between the two silently changes the argument's meaning
depending on which entrypoint a user reads or runs, so this parses both
files' DeclareLaunchArgument calls, as motion_profile_contract_test.py
already does for one of them, rather than executing either launch file.
"""

import ast
from pathlib import Path
import unittest

GAZEBO_PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = GAZEBO_PACKAGE.parent
GAZEBO_LAUNCH = GAZEBO_PACKAGE / "launch" / "rosmaster_gazebo_fortress.launch.py"
BRINGUP_LAUNCH = (
    REPOSITORY_ROOT / "yahboom_rosmaster_bringup" / "launch" / "rosmaster_x3_sim.launch.py")
SHARED_ARGUMENTS = (
    "sensor_profile", "sensor_seed", "cloud_strip_nan", "cloud_decimation",
    "motion_profile",
)
FIELDS = ("default_value", "description", "choices")


def declared_arguments(launch_file):
    """
    Return {name: {field: value}} for every DeclareLaunchArgument(...) call.

    A field whose value is not a literal (a variable, or an expression such as
    the platform-dependent defaults elsewhere in these files) is left out
    rather than failing the parse; none of the shared arguments this test
    checks use one.
    """
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))
    arguments = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        values = {}
        for field in FIELDS:
            if field not in keywords:
                continue
            try:
                values[field] = ast.literal_eval(keywords[field])
            except ValueError:
                pass
        arguments[node.args[0].value] = values
    return arguments


class TestSharedLaunchArguments(unittest.TestCase):
    """The bringup entrypoint must declare these exactly like the file it includes."""

    @classmethod
    def setUpClass(cls):
        cls.gazebo_arguments = declared_arguments(GAZEBO_LAUNCH)
        cls.bringup_arguments = declared_arguments(BRINGUP_LAUNCH)

    def test_the_shared_arguments_are_declared_in_both_files(self):
        for name in SHARED_ARGUMENTS:
            with self.subTest(argument=name):
                self.assertIn(name, self.gazebo_arguments)
                self.assertIn(name, self.bringup_arguments)

    def test_the_shared_arguments_default_description_and_choices_match(self):
        for name in SHARED_ARGUMENTS:
            with self.subTest(argument=name):
                self.assertEqual(
                    self.bringup_arguments[name], self.gazebo_arguments[name])


if __name__ == "__main__":
    unittest.main()
