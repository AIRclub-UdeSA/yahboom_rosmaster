#!/usr/bin/env python3
"""
A bare rclpy node that does nothing, on sim time or on wall time.

``measure_python_node_cpu.py`` starts one of each beside the simulator. Their
CPU difference is what an rclpy node pays just to follow the 1 kHz ``/clock``,
with no callback of its own and no numpy.

    python3 tools/idle_node.py --sim-time true|false [--timer-hz HZ]

``--timer-hz`` adds a timer, so the same pair also shows a node that already
wakes at its own rate. Nothing here is installed as a node.
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    """Spin until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--sim-time", choices=("true", "false"), required=True)
    parser.add_argument("--name", default="idle_node")
    parser.add_argument("--timer-hz", type=float, default=0.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node(args.name, parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, args.sim_time == "true")])
    if args.timer_hz > 0.0:
        node.create_timer(1.0 / args.timer_hz, lambda: None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
