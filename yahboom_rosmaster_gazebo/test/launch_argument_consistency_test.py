#!/usr/bin/env python3
"""
Cross-file consistency of the launch arguments both simulator entrypoints share.

rosmaster_x3_sim.launch.py (the bringup entrypoint) declares its own copy of
every argument it forwards to rosmaster_gazebo_fortress.launch.py, the file it
includes. A default, description or set of choices that drifts between the
two silently changes the argument's meaning depending on which entrypoint a
user reads or runs, so this parses both files' DeclareLaunchArgument calls,
as motion_profile_contract_test.py already does for one of them, rather than
executing either launch file, and compares every argument name declared in
both: the intersection, not a fixed list, so a new shared argument (such as
#54's spawn_x, spawn_y and spawn_yaw) is covered without editing this test.
"""

import ast
from pathlib import Path
import unittest

GAZEBO_PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = GAZEBO_PACKAGE.parent
GAZEBO_LAUNCH = GAZEBO_PACKAGE / "launch" / "rosmaster_gazebo_fortress.launch.py"
BRINGUP_LAUNCH = (
    REPOSITORY_ROOT / "yahboom_rosmaster_bringup" / "launch" / "rosmaster_x3_sim.launch.py")
FIELDS = ("default_value", "description", "choices")

# Arguments the two files are allowed to declare differently, and why. An
# argument here is still required to be declared in both files (see
# test_documented_differences_still_apply below); only the strict-equality
# check is skipped for it. Naming an exception here, instead of leaving the
# argument out of the comparison silently, means a reviewer sees the excuse
# and a future difference in anything else about the argument still fails.
DOCUMENTED_DIFFERENCES = {
    "world": (
        "the gazebo file needs a default it can resolve when launched on its "
        "own, so it computes an absolute path (not a literal, so already "
        "skipped by field) and declares no description; the bringup file's "
        "default is the short relative name a user types, which the gazebo "
        "file's own path resolution (_launch_gazebo_server) accepts the same "
        "way"),
    "motion_bias": (
        "deliberately different defaults: the gazebo file defaults it on, "
        "since that is the uncalibrated mecanum base the motion-profile "
        "tests are written against, while the bringup entrypoint defaults it "
        "off for a plain, undrifted run"),
}


def declared_arguments(launch_file):
    """
    Return {name: {field: value}} for every DeclareLaunchArgument(...) call.

    A field whose value is not a literal (a variable, or an expression such as
    the platform-dependent defaults elsewhere in these files) is left out
    rather than failing the parse.
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
    """Every argument both files declare must match, or be a documented exception."""

    @classmethod
    def setUpClass(cls):
        cls.gazebo_arguments = declared_arguments(GAZEBO_LAUNCH)
        cls.bringup_arguments = declared_arguments(BRINGUP_LAUNCH)
        cls.shared = sorted(set(cls.gazebo_arguments) & set(cls.bringup_arguments))

    def test_the_parser_still_finds_the_arguments_the_two_files_share(self):
        # A parser regression would make every other test in this file
        # vacuously pass, so pin a lower bound on what it found.
        self.assertGreaterEqual(len(self.shared), 13)
        self.assertIn("cloud_decimation", self.shared)
        self.assertIn("spawn_x", self.shared)

    def test_documented_differences_are_still_shared_and_still_differ(self):
        for name, reason in DOCUMENTED_DIFFERENCES.items():
            with self.subTest(argument=name):
                self.assertIn(
                    name, self.shared,
                    f"{name} is no longer declared in both files; drop its "
                    "documented exception")
                self.assertTrue(reason.strip())
                self.assertNotEqual(
                    self.bringup_arguments[name], self.gazebo_arguments[name],
                    f"{name} no longer differs between the two files; drop "
                    "its documented exception so this test enforces the match")

    def test_every_other_shared_argument_matches_exactly(self):
        for name in self.shared:
            if name in DOCUMENTED_DIFFERENCES:
                continue
            with self.subTest(argument=name):
                self.assertEqual(
                    self.bringup_arguments[name], self.gazebo_arguments[name])


if __name__ == "__main__":
    unittest.main()
