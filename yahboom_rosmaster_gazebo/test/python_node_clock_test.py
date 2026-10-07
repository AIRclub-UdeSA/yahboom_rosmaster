#!/usr/bin/env python3
"""
Check which Python nodes of the Fortress launch file follow the 1 kHz /clock.

An rclpy node with use_sim_time true wakes on every tick of /clock, which costs
about 40% of a core in Python (#75). A node that stamps its output from its
input messages, or only needs elapsed wall time, must not pay that. This parses
rosmaster_gazebo_fortress.launch.py, finds every Node whose executable is a
Python script, and checks that its use_sim_time parameter is either absent
(rclpy's default is wall time) or the literal False, unless the node is listed
in SIM_TIME_PYTHON_NODES below.

ground_truth_tf.py is not in that list: it takes every transform stamp from the
incoming /ground_truth/odom header, so it runs on wall time (#75, first step).
SIM_TIME_PYTHON_NODES is the work still open under #75; the check that no
simulator Python node follows the clock is this test with the list empty.
"""

import ast
from pathlib import Path
import unittest

GAZEBO_PACKAGE = Path(__file__).resolve().parents[1]
GAZEBO_LAUNCH = GAZEBO_PACKAGE / "launch" / "rosmaster_gazebo_fortress.launch.py"

# Python nodes that still follow /clock, until #75's later steps deal with them.
SIM_TIME_PYTHON_NODES = {
    "camera_adapter.py": "#75: still on sim time; a later step",
    "calculated_odometry.py": "#75: still on sim time; a later step",
}


def python_nodes(launch_file):
    """
    Return {executable: [use_sim_time AST or None, ...]} for every Python Node.

    The value is the expression written under the "use_sim_time" key of the
    Node's parameters, or None when the node sets no such parameter. A Node
    appearing in several places contributes one entry per call.
    """
    tree = ast.parse(launch_file.read_text(encoding="utf-8"))
    nodes = {}
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        if not isinstance(call.func, ast.Name) or call.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        executable = keywords.get("executable")
        if not isinstance(executable, ast.Constant) or not str(
                executable.value).endswith(".py"):
            continue
        setting = None
        for dictionary in ast.walk(keywords.get("parameters", ast.Tuple(elts=[]))):
            if not isinstance(dictionary, ast.Dict):
                continue
            for key, value in zip(dictionary.keys, dictionary.values):
                if isinstance(key, ast.Constant) and key.value == "use_sim_time":
                    setting = value
        nodes.setdefault(executable.value, []).append(setting)
    return nodes


def follows_sim_time(setting):
    """Return True unless the parameter is absent or the literal False."""
    if setting is None:
        return False
    return not (isinstance(setting, ast.Constant) and setting.value is False)


class PythonNodeClock(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.nodes = python_nodes(GAZEBO_LAUNCH)

    def test_the_parser_still_finds_the_python_nodes(self):
        # A parser regression would make every other test here pass vacuously.
        for executable in (
                "ground_truth_tf.py", "camera_adapter.py", "calculated_odometry.py",
                "sensor_qos_relay.py", "imu_raw_relay.py"):
            self.assertIn(executable, self.nodes)

    def test_ground_truth_tf_runs_on_wall_time(self):
        settings = self.nodes["ground_truth_tf.py"]
        self.assertEqual(len(settings), 1)
        self.assertTrue(
            isinstance(settings[0], ast.Constant) and settings[0].value is False,
            "ground_truth_tf.py must set use_sim_time to the literal False: it "
            "stamps from /ground_truth/odom and would otherwise wake on every "
            "/clock tick (#75)")

    def test_only_the_listed_python_nodes_follow_the_clock(self):
        followers = {
            executable for executable, settings in self.nodes.items()
            if any(follows_sim_time(setting) for setting in settings)}
        self.assertEqual(
            followers - set(SIM_TIME_PYTHON_NODES), set(),
            "these Python nodes follow the 1 kHz /clock (#75): run them on wall "
            "time, or list them in SIM_TIME_PYTHON_NODES with the reason")

    def test_the_allowlist_has_no_stale_entries(self):
        followers = {
            executable for executable, settings in self.nodes.items()
            if any(follows_sim_time(setting) for setting in settings)}
        self.assertEqual(
            set(SIM_TIME_PYTHON_NODES) - followers, set(),
            "remove nodes from SIM_TIME_PYTHON_NODES once they run on wall time")


if __name__ == "__main__":
    unittest.main()
