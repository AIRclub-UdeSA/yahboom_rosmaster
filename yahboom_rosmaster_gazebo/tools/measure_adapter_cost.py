#!/usr/bin/env python3
r"""
Measure the camera adapter's CPU and the depth image's latency, alternating variants.

Each run starts one headless simulator on empty.world under
``sensor_profile:=physical``, lets it settle, and then over a window reads:

* the CPU of the camera adapter and of the Gazebo server, as percent of one
  core, from /proc (ticks used over wall time);
* the depth image's latency, as sim time at receipt minus ``header.stamp``, and
  its rate on stamps;
* the real-time factor, as sim seconds over wall seconds.

``--no-user-site`` runs the simulator with ``PYTHONNOUSERSITE=1``, which hides
numpy packages in ``~/.local`` and so loads the distribution's own numpy, as the
CI image does. The report names the numpy each figure used.

Variants are workspaces, for example a checkout of ``main`` and a feature
branch. They run alternately (A, B, A, B, ...) so a change in the machine's load
touches both alike. The report gives n, the median and the range of each figure
per variant, on the render path named (``gpu`` or ``llvmpipe``, CI's software
rendering pinned to four CPUs).

Run it with ROS sourced, from a source tree, with no simulator running::

    python3 tools/measure_adapter_cost.py --render gpu --runs 5 \
        --variant main=~/Documents/rosmaster_ws \
        --variant step7=~/Documents/rosmaster_ws_step7

Nothing here is installed as a node.
"""

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_run import Simulator, isolated_environment  # noqa: E402

DEPTH_TOPIC = "/cam_1/depth/image_raw"
LAUNCH_ARGUMENTS = (
    "headless:=true", "rviz:=false", "use_sim_time:=true", "world:=empty.world",
    "sensor_profile:={profile}", "sensor_seed:={seed}", "motion_bias:=false")
FIRST_FRAME_TIMEOUT_S = {"gpu": 90.0, "llvmpipe": 240.0}


class LatencyProbe:
    """Record the depth image's latency and stamps on the ROS clock, on sim time."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node(
            "adapter_cost_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.latencies_s = []
        self.stamps_s = []
        self.window_open = False
        self.sim_start = self.wall_start = None
        self.node.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)

    def _now_s(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_depth(self, message):
        self.frames_seen = getattr(self, "frames_seen", 0) + 1
        if not self.window_open:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.latencies_s.append(self._now_s() - stamp)
        self.stamps_s.append(stamp)

    def spin_for(self, seconds):
        """Spin for ``seconds`` of wall time."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_frames(self, timeout_s, count=5):
        """Return whether ``count`` depth images arrived within ``timeout_s``."""
        self.frames_seen = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.frames_seen >= count:
                return True
        return False

    def open_window(self):
        """Start recording."""
        self.latencies_s, self.stamps_s = [], []
        self.sim_start, self.wall_start = self._now_s(), time.monotonic()
        self.window_open = True

    def close_window(self):
        """Stop recording and return sim seconds and wall seconds of the window."""
        self.window_open = False
        return self._now_s() - self.sim_start, time.monotonic() - self.wall_start

    def shutdown(self):
        """Release the node."""
        self.node.destroy_node()
        self.rclpy.shutdown()


def numpy_version(environment):
    """Return the version of the numpy a child process with ``environment`` imports."""
    merged = dict(os.environ)
    merged.update(environment)
    return subprocess.run(
        [sys.executable, "-c", "import numpy; print(numpy.__version__)"],
        env=merged, capture_output=True, text=True, check=True).stdout.strip()


def _cpu_ticks(simulator, marker):
    """Return the CPU ticks of the session's processes whose command has ``marker``."""
    return sum(
        ticks for command, ticks in simulator.processes().values() if marker in command)


def _percent(ticks, wall_s):
    return 100.0 * ticks / os.sysconf("SC_CLK_TCK") / wall_s


def one_run(label, workspace, args, index):
    """Run one simulator and return its figures, or a failure record."""
    seed = 1000 + index
    domain = 60 + (os.getpid() + index) % 150
    environment = isolated_environment(domain, f"adapter_cost_{os.getpid()}_{index}")
    if args.no_user_site:
        environment["PYTHONNOUSERSITE"] = "1"
    simulator = Simulator(
        workspace,
        Path(args.output_dir) / f"{args.render}_{label}_{index}.log",
        [text.format(profile=args.profile, seed=seed) for text in LAUNCH_ARGUMENTS],
        environment, render=args.render)
    record = {
        "variant": label, "run": index, "render": args.render,
        "numpy": numpy_version(environment)}
    os.environ.update(isolated_environment(domain, "unused"))
    probe = None
    try:
        simulator.start()
        probe = LatencyProbe()
        if not probe.wait_for_frames(FIRST_FRAME_TIMEOUT_S[args.render]):
            record["error"] = "no depth image within the start-up timeout"
            return record
        probe.spin_for(args.settle_s)
        adapter_before = _cpu_ticks(simulator, "camera_adapter")
        server_before = _cpu_ticks(simulator, "ign gazebo")
        everything_before = sum(t for _, t in simulator.processes().values())
        probe.open_window()
        probe.spin_for(args.window_s)
        sim_s, wall_s = probe.close_window()
        record.update({
            "adapter_cpu_percent": _percent(
                _cpu_ticks(simulator, "camera_adapter") - adapter_before, wall_s),
            "server_cpu_percent": _percent(
                _cpu_ticks(simulator, "ign gazebo") - server_before, wall_s),
            "session_cpu_percent": _percent(
                sum(t for _, t in simulator.processes().values()) - everything_before,
                wall_s),
            "real_time_factor": sim_s / wall_s,
            "depth_latency_ms_median": 1000.0 * statistics.median(probe.latencies_s),
            "depth_latency_ms_p95": 1000.0 * sorted(probe.latencies_s)[
                int(0.95 * (len(probe.latencies_s) - 1))],
            "depth_frames": len(probe.stamps_s),
            "depth_rate_hz_on_stamps": (len(probe.stamps_s) - 1) / (
                probe.stamps_s[-1] - probe.stamps_s[0]),
            "window_wall_s": wall_s,
        })
    finally:
        if probe is not None:
            probe.shutdown()
        record["shutdown"] = simulator.stop()
    return record


def summarize(records, key):
    """Return n, median, min and max of ``key`` over the records that have it."""
    values = [record[key] for record in records if key in record]
    if not values:
        return None
    return {
        "n": len(values),
        "median": round(statistics.median(values), 3),
        "min": round(min(values), 3),
        "max": round(max(values), 3),
    }


def main():
    """Run the alternating measurement and print a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--variant", action="append", required=True,
                        metavar="LABEL=WORKSPACE",
                        help="a built workspace to measure; give at least one")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--profile", choices=("physical", "ideal"), default="physical")
    parser.add_argument("--runs", type=int, default=5, help="runs per variant")
    parser.add_argument("--settle-s", type=float, default=20.0,
                        help="wall seconds to wait after the first frame")
    parser.add_argument("--window-s", type=float, default=30.0,
                        help="wall seconds measured")
    parser.add_argument("--no-user-site", action="store_true",
                        help="hide ~/.local packages: use the distribution's numpy")
    parser.add_argument("--output-dir", default="adapter_cost_logs")
    parser.add_argument("--output", help="write the JSON report here")
    args = parser.parse_args()

    variants = []
    for text in args.variant:
        label, _, path = text.partition("=")
        variants.append((label, Path(path).expanduser().resolve()))

    records = []
    for index in range(args.runs):
        for label, workspace in variants:
            print(f"run {index + 1}/{args.runs} {label} ({args.render})", flush=True)
            record = one_run(label, workspace, args, index)
            print(json.dumps(record), flush=True)
            records.append(record)

    report = {
        "render": args.render, "profile": args.profile,
        "numpy": sorted({record["numpy"] for record in records}),
        "runs": records, "summary": {}}
    for label, _ in variants:
        mine = [record for record in records if record["variant"] == label]
        report["summary"][label] = {
            key: summarize(mine, key) for key in (
                "adapter_cpu_percent", "server_cpu_percent", "session_cpu_percent",
                "real_time_factor", "depth_latency_ms_median", "depth_latency_ms_p95",
                "depth_rate_hz_on_stamps")}
        report["summary"][label]["failed_runs"] = sum("error" in r for r in mine)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
