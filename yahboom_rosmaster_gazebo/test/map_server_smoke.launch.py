#!/usr/bin/env python3
"""
Launch every occupancy map under maps/ through real nav2_map_server nodes.

Auto-discovers maps/*.yaml at launch-generation time, so a new map gets this
coverage with zero extra wiring here or in the CI test filter -- see #40's
discussion of why the explicit-target-per-map pattern (world_smoke.launch.py)
is the wrong model to copy for this one.
"""

import os
import re
import sys
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shutdown_asserts import assert_clean_shutdown  # noqa: E402


# A small spread on process creation, not a correctness requirement: the
# probe (map_server_smoke_probe.py) drives and retries each node's lifecycle
# itself, so it tolerates slow service responses under load regardless. This
# just cuts how often a retry is needed by not starting every map_server at
# the exact same instant.
STAGGER_PERIOD_S = 0.3


def _node_name(yaml_path):
    """Build a ROS-legal, unique node name derived from the map's filename."""
    return "map_server_" + re.sub(r"[^a-zA-Z0-9_]", "_", yaml_path.stem)


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory("yahboom_rosmaster_gazebo")
    maps_dir = Path(package_share) / "maps"
    yaml_paths = sorted(maps_dir.glob("*.yaml"))
    node_names = [_node_name(path) for path in yaml_paths]

    staggered_map_servers = [
        TimerAction(
            period=index * STAGGER_PERIOD_S,
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name=name,
                    parameters=[{
                        "yaml_filename": str(yaml_path), "use_sim_time": False}],
                    # Every instance defaults to publishing /map; keep each on
                    # its own topic so N concurrent nodes don't crosstalk on
                    # the shared name.
                    remappings=[("map", f"{name}/map")],
                    output="screen",
                ),
            ],
        )
        for index, (name, yaml_path) in enumerate(zip(node_names, yaml_paths))
    ]
    probe = Node(
        package="yahboom_rosmaster_gazebo",
        executable="map_server_smoke_probe.py",
        parameters=[{"node_names": node_names}],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", str(10 + os.getpid() % 211)),
        *staggered_map_servers,
        probe,
        launch_testing.actions.ReadyToTest(),
    ]), {"probe": probe}


class TestMapServerSmoke(unittest.TestCase):
    """Require the probe to confirm every map's node reaches ACTIVE."""

    def test_probe_passes(self, proc_info, probe):
        proc_info.assertWaitForStartup(probe, timeout=15)
        proc_info.assertWaitForShutdown(probe, timeout=100)
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Reject signal escalation or orphan-prone process exits."""

    def test_all_processes_exit_cleanly(self, proc_info):
        assert_clean_shutdown(proc_info)
