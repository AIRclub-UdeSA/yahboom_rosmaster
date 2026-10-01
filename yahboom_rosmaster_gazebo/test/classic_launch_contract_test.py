#!/usr/bin/env python3
"""
Check that the Gazebo Classic (macOS) launch file limits commands with the bias off.

yahboom_robostack_M_silycon/launch/simulation.launch.py runs only where Gazebo
Classic is installed, so no CI job can launch it. This builds its launch
description with every launch_ros Node replaced by a recorder, runs the
OpaqueFunctions against a context, and checks what would have been started:

* exactly one cmd_vel_watchdog, with no condition, for motion_bias true and false;
* its motion_bias_file set when the bias is on and empty (relay unmodified) when
  it is off;
* planar_move reading the watchdog's output: the xacro gets
  cmd_vel_topic:=cmd_vel_classic in both cases, which is the topic the
  watchdog publishes.

Without the watchdog on the planar_move path, motion_bias:=false skipped the
+-1.0 m/s and +-5.0 rad/s clamp and the 0.5 s zero-on-silence timeout (#61
review). Needs a sourced workspace, since the launch file resolves
yahboom_rosmaster_gazebo and yahboom_rosmaster_description through ament.
"""

import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch.substitutions import Command, TextSubstitution

LAUNCH_FILE = (
    Path(__file__).resolve().parents[2]
    / "yahboom_robostack_M_silycon" / "launch" / "simulation.launch.py")


class RecordedNode:
    """Stand in for launch_ros.actions.Node and keep the keyword arguments."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs


def command_text(command):
    """Join the literal text of a Command substitution without running it."""
    return "".join(
        part.text for part in command.command if isinstance(part, TextSubstitution))


def collect_nodes(motion_bias):
    """Return the recorded Nodes of the launch description for one bias setting."""
    spec = importlib.util.spec_from_file_location("classic_simulation_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.Node = RecordedNode

    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["motion_bias"] = motion_bias
    context.launch_configurations["use_sim_time"] = "true"
    nodes = []
    for entity in description.entities:
        if isinstance(entity, RecordedNode):
            nodes.append(entity)
        elif isinstance(entity, OpaqueFunction):
            nodes.extend(
                result for result in entity.execute(context) or []
                if isinstance(result, RecordedNode))
    return nodes


class ClassicLaunchContract(unittest.TestCase):

    def watchdog(self, motion_bias):
        nodes = [
            node for node in collect_nodes(motion_bias)
            if node.kwargs.get("executable") == "cmd_vel_watchdog.py"]
        self.assertEqual(len(nodes), 1, f"motion_bias={motion_bias}: one watchdog expected")
        return nodes[0].kwargs

    def planar_move_topic(self, motion_bias):
        for node in collect_nodes(motion_bias):
            description = node.kwargs.get("parameters", [{}])[0].get("robot_description")
            if description is not None:
                (command,) = description.value
                self.assertIsInstance(command, Command)
                text = command_text(command)
                self.assertIn(" cmd_vel_topic:=", text)
                return text.split(" cmd_vel_topic:=")[1].split()[0]
        self.fail("no robot_state_publisher node")

    def test_watchdog_always_runs(self):
        for motion_bias in ("true", "false"):
            with self.subTest(motion_bias=motion_bias):
                self.assertIsNone(self.watchdog(motion_bias).get("condition"))

    def test_bias_file_follows_motion_bias(self):
        biased = self.watchdog("true")["parameters"][0]
        unbiased = self.watchdog("false")["parameters"][0]
        self.assertTrue(biased["motion_bias_file"].endswith("motion_bias.yaml"))
        self.assertEqual(unbiased["motion_bias_file"], "")

    def test_watchdog_clamps_in_both_modes(self):
        for motion_bias in ("true", "false"):
            parameters = self.watchdog(motion_bias)["parameters"][0]
            with self.subTest(motion_bias=motion_bias):
                for name in ("linear_x_limit", "linear_y_limit", "angular_z_limit"):
                    self.assertGreater(parameters[name], 0.0)

    def test_planar_move_reads_the_watchdog_output(self):
        for motion_bias in ("true", "false"):
            parameters = self.watchdog(motion_bias)["parameters"][0]
            with self.subTest(motion_bias=motion_bias):
                self.assertEqual(parameters["input_topic"], "/cmd_vel")
                self.assertEqual(
                    "/" + self.planar_move_topic(motion_bias), parameters["output_topic"])
                self.assertEqual(self.planar_move_topic(motion_bias), "cmd_vel_classic")


if __name__ == "__main__":
    unittest.main()
