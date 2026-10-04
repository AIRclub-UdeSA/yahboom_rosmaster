#!/usr/bin/env python3
r"""
Break down where the Python nodes' CPU goes in a default headless launch.

Each run starts one simulator, lets it settle, and over a window of wall time
reads from /proc, per process and per thread, the CPU ticks used, as percent of
one core. It reports:

* every process of the launch session, labelled by its script or executable, so
  the Python nodes (``camera_adapter.py``, ``calculated_odometry.py``, ...)
  appear beside Gazebo's server and the bridges;
* which nodes subscribe to ``/clock`` (``ros2 topic info /clock -v``), the
  ones that pay the sim-time tax;
* the camera adapter's CPU split by thread name (the executor thread that runs
  Python and numpy, against the Fast DDS threads), and its CPU per sim second;
* with ``--idle-nodes``, two bare rclpy nodes (``idle_node.py``), one on sim
  time and one on wall time, whose difference is the tax with no callback and
  no numpy in the way;
* the point cloud's latency (sim time at receipt minus ``header.stamp``) and its
  rate on stamps, and the number of messages on each ``--count-topic`` over the
  same window, so a hop that drops messages shows as a shortfall;
* the real-time factor, as sim seconds over wall seconds.

Variants are built workspaces, optionally with extra launch arguments, and they
run alternately (A, B, A, B, ...) so a change in the machine's load touches
both alike. The report gives n, the median and the range per figure and
variant, with the numpy each run used.

The cleanup is ``scripts/clean_sim.sh`` over every variant's workspace (the
anchored patterns of AGENTS.md), not ``sim_run.clean_up``, whose patterns do
not cover a workspace such as ``rosmaster_ws_cpu``. Run it with ROS sourced,
from a source tree, with no other simulator on the machine::

    python3 tools/measure_python_node_cpu.py --render gpu --runs 5 \
        --world cafe.world --profile physical --no-user-site --idle-nodes \
        --variant main=~/Documents/rosmaster_ws_cpu \
        --output cpu_cafe_physical_gpu.json

    # Another variant with a launch argument of its own, for example a
    # prototype that runs the adapter on wall time:
    #   --variant wall=~/Documents/rosmaster_ws_cpu \
    #   --variant-arg wall:camera_adapter_sim_time:=false

``--render llvmpipe`` reproduces CI's software rendering pinned to four CPUs
(the CPU figures then use a real-time factor of about 0.49, so read
``*_per_sim_s`` for what a node costs per simulated second). ``--no-user-site``
hides ``~/.local`` and so loads the distribution's numpy 1.21.5, as CI does.
No profiler is installed or needed: ``perf`` is blocked by
``kernel.perf_event_paranoid``, and the thread split stands in for it.

Nothing here is installed as a node.
"""

import argparse
import collections
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import sim_run  # noqa: E402
from measure_adapter_cost import numpy_version, summarize  # noqa: E402
from sim_run import CLOCK_TICKS, Simulator, isolated_environment  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
CLOUD_TOPIC = "/cam_1/depth/color/points"
LAUNCH_ARGUMENTS = (
    "headless:=true", "rviz:=false", "use_sim_time:=true", "world:={world}",
    "sensor_profile:={profile}", "sensor_seed:={seed}", "motion_bias:=false")
FIRST_FRAME_TIMEOUT_S = {"gpu": 90.0, "llvmpipe": 240.0}
SCRIPT_INTERPRETERS = ("python", "ruby", "bash", "sh", "xvfb-run", "taskset")


def make_cleanup(workspaces):
    """Return a replacement for ``sim_run.clean_up`` that runs clean_sim.sh."""
    def clean_up():
        subprocess.run(
            ["bash", str(REPOSITORY_ROOT / "scripts" / "clean_sim.sh"),
             *[str(path) for path in workspaces]], check=False)
    return clean_up


def label_of(command):
    """Return a short label for a process: its script, or its executable."""
    words = command.split()
    if "idle_node.py" in command and "--sim-time" in words:
        return f"idle_node(sim_time={words[words.index('--sim-time') + 1]})"
    for word in words:
        base = os.path.basename(word)
        if word.startswith("-"):
            continue
        if any(base.startswith(name) for name in SCRIPT_INTERPRETERS) \
                and not base.endswith(".py"):
            continue
        return base
    return command[:40]


def thread_ticks(pid):
    """Return {thread name: CPU ticks} for a process, summed over same-named threads."""
    found = collections.Counter()
    for task in Path(f"/proc/{pid}/task").iterdir():
        try:
            stat = (task / "stat").read_text()
            name = stat[stat.index("(") + 1:stat.rindex(")")]
            fields = stat[stat.rindex(")") + 2:].split()
            found[name] += int(fields[11]) + int(fields[12])
        except (OSError, ValueError, IndexError):
            continue
    return found


def process_ticks(extra_pids):
    """Return {pid: (label, ticks)} of the session's processes plus ``extra_pids``."""
    found = {}
    for pid in extra_pids:
        try:
            command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(
                b"\0", b" ").decode(errors="replace").strip()
            stat = Path(f"/proc/{pid}/stat").read_text()
            fields = stat[stat.rindex(")") + 2:].split()
            found[pid] = (command, int(fields[11]) + int(fields[12]))
        except (OSError, ValueError, IndexError):
            continue
    return found


def by_label(snapshot):
    """Sum a ``{pid: (command, ticks)}`` snapshot by label."""
    total = collections.Counter()
    for command, ticks in snapshot.values():
        total[label_of(command)] += ticks
    return total


def percent(ticks, wall_s):
    """Return ``ticks`` of CPU over ``wall_s`` seconds as percent of one core."""
    return 100.0 * ticks / CLOCK_TICKS / wall_s


def clock_subscribers(environment, workspace):
    """Return the node names that subscribe to /clock, from the ROS graph."""
    command = sim_run._ros_shell("ros2 topic info /clock -v", workspace)
    merged = dict(os.environ)
    merged.update(environment)
    result = subprocess.run(
        ["bash", "-c", command], env=merged, capture_output=True, text=True,
        timeout=60, check=False)
    names, current = [], None
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.startswith("Node name:"):
            current = line.split(":", 1)[1].strip()
        elif line.startswith("Endpoint type:") and current is not None:
            if line.endswith("SUBSCRIPTION"):
                names.append(current)
            current = None
    return sorted(set(names))


class CloudProbe:
    """Record the cloud's latency and stamps on sim time, and count other topics."""

    def __init__(self, count_topics):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node(
            "python_node_cpu_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.latencies_s, self.stamps_s = [], []
        self.counts = {topic: 0 for topic in count_topics}
        self.window_open = False
        self.seen = 0
        self.sim_start = self.wall_start = None
        self.node.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data)
        for topic in count_topics:
            self.node.create_subscription(
                PointCloud2, topic, self._counter(topic), qos_profile_sensor_data)

    def _now_s(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _counter(self, topic):
        def count(_message):
            if self.window_open:
                self.counts[topic] += 1
        return count

    def _on_cloud(self, message):
        self.seen += 1
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

    def wait_for_clouds(self, timeout_s, count=3):
        """Return whether ``count`` clouds arrived within ``timeout_s``."""
        self.seen = 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.seen >= count:
                return True
        return False

    def open_window(self):
        """Start recording."""
        self.latencies_s, self.stamps_s = [], []
        self.counts = dict.fromkeys(self.counts, 0)
        self.sim_start, self.wall_start = self._now_s(), time.monotonic()
        self.window_open = True

    def close_window(self):
        """Stop recording and return the sim and wall seconds of the window."""
        self.window_open = False
        return self._now_s() - self.sim_start, time.monotonic() - self.wall_start

    def shutdown(self):
        """Release the node."""
        self.node.destroy_node()
        self.rclpy.shutdown()


def start_idle_nodes(workspace, environment):
    """Start an idle rclpy node on sim time and one on wall time; return their pids."""
    pids = {}
    merged = dict(os.environ)
    merged.update(environment)
    script = Path(__file__).resolve().parent / "idle_node.py"
    for sim_time in ("true", "false"):
        name = f"idle_sim_{sim_time}"
        command = sim_run._ros_shell(
            f"exec python3 {script} --sim-time {sim_time} --name {name}", workspace)
        process = subprocess.Popen(
            ["bash", "-c", command], env=merged, stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        pids[process.pid] = process
    return pids


def stop_idle_nodes(processes):
    """Stop the idle nodes by pid (each is its own session leader)."""
    for pid, process in processes.items():
        try:
            os.killpg(pid, 2)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(pid, 9)


def one_run(label, workspace, extra_arguments, args, index):
    """Run one simulator and return its figures, or a failure record."""
    seed = 1000 + index
    domain = 60 + (os.getpid() + index) % 150
    environment = isolated_environment(domain, f"python_cpu_{os.getpid()}_{index}")
    if args.no_user_site:
        environment["PYTHONNOUSERSITE"] = "1"
    arguments = [
        text.format(world=args.world, profile=args.profile, seed=seed)
        for text in LAUNCH_ARGUMENTS] + list(extra_arguments)
    log = Path(args.output_dir) / f"{args.render}_{label}_{index}.log"
    simulator = Simulator(workspace, log, arguments, environment, render=args.render)
    record = {
        "variant": label, "run": index, "render": args.render, "world": args.world,
        "profile": args.profile, "numpy": numpy_version(environment),
        "launch_arguments": arguments}
    os.environ.update(isolated_environment(domain, "unused"))
    probe, idle = None, {}
    try:
        simulator.start()
        probe = CloudProbe(args.count_topic)
        if not probe.wait_for_clouds(FIRST_FRAME_TIMEOUT_S[args.render]):
            record["error"] = "no cloud within the start-up timeout"
            return record
        if args.idle_nodes:
            idle = start_idle_nodes(workspace, environment)
        probe.spin_for(args.settle_s)
        record["clock_subscribers"] = clock_subscribers(environment, workspace)

        def snapshot():
            merged = {pid: value for pid, value in simulator.processes().items()}
            merged.update(process_ticks(idle))
            return merged

        adapter_pids = [
            pid for pid, (command, _) in simulator.processes().items()
            if "camera_adapter" in command]
        threads_before = {pid: thread_ticks(pid) for pid in adapter_pids}
        before = by_label(snapshot())
        probe.open_window()
        probe.spin_for(args.window_s)
        sim_s, wall_s = probe.close_window()
        after = by_label(snapshot())
        threads_after = {pid: thread_ticks(pid) for pid in adapter_pids}

        record["processes_cpu_percent"] = {
            name: percent(after[name] - before.get(name, 0), wall_s) for name in after}
        record["processes_cpu_percent_per_sim_s"] = {
            name: percent(after[name] - before.get(name, 0), sim_s) for name in after}
        record["camera_adapter_threads_cpu_percent"] = {
            name: percent(
                sum(threads_after[pid][name] - threads_before[pid].get(name, 0)
                    for pid in adapter_pids), wall_s)
            for name in {n for pid in adapter_pids for n in threads_after[pid]}}
        record["session_cpu_percent"] = sum(record["processes_cpu_percent"].values())
        record["real_time_factor"] = sim_s / wall_s
        record["window_sim_s"], record["window_wall_s"] = sim_s, wall_s
        if probe.stamps_s:
            ordered = sorted(probe.latencies_s)
            record.update({
                "cloud_latency_ms_median": 1000.0 * statistics.median(ordered),
                "cloud_latency_ms_p95": 1000.0 * ordered[int(0.95 * (len(ordered) - 1))],
                "cloud_latency_ms_min": 1000.0 * ordered[0],
                "cloud_latency_ms_max": 1000.0 * ordered[-1],
                "cloud_count": len(probe.stamps_s),
                "cloud_rate_hz_on_stamps": (len(probe.stamps_s) - 1) / (
                    probe.stamps_s[-1] - probe.stamps_s[0]),
            })
        record["topic_counts"] = dict(probe.counts)
    finally:
        if probe is not None:
            probe.shutdown()
        stop_idle_nodes(idle)
        record["shutdown"] = simulator.stop()
    return record


def flat_figures(record):
    """Return a record's figures as one flat {name: number} mapping."""
    flat = {key: record[key] for key in record if isinstance(record[key], float)}
    for group in ("processes_cpu_percent", "processes_cpu_percent_per_sim_s",
                  "camera_adapter_threads_cpu_percent"):
        for name, value in record.get(group, {}).items():
            flat[f"{group}.{name}"] = value
    for topic, count in record.get("topic_counts", {}).items():
        flat[f"topic_count.{topic}"] = float(count)
    if "cloud_count" in record:
        flat["cloud_count"] = float(record["cloud_count"])
    return flat


def main():
    """Run the alternating measurement and print a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--variant", action="append", required=True,
                        metavar="LABEL=WORKSPACE", help="a built workspace; at least one")
    parser.add_argument("--variant-arg", action="append", default=[],
                        metavar="LABEL:ARG:=VALUE",
                        help="an extra launch argument for one variant (repeatable)")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--world", default="empty.world")
    parser.add_argument("--profile", choices=("physical", "ideal"), default="physical")
    parser.add_argument("--runs", type=int, default=5, help="runs per variant")
    parser.add_argument("--settle-s", type=float, default=20.0)
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--idle-nodes", action="store_true",
                        help="also run a sim-time and a wall-time idle rclpy node")
    parser.add_argument("--count-topic", action="append", default=[], metavar="TOPIC",
                        help="also count PointCloud2 messages on TOPIC in the window")
    parser.add_argument("--no-user-site", action="store_true",
                        help="hide ~/.local packages: use the distribution's numpy")
    parser.add_argument("--output-dir", default="python_node_cpu_logs")
    parser.add_argument("--output", help="write the JSON report here")
    args = parser.parse_args()

    variants = []
    for text in args.variant:
        label, _, path = text.partition("=")
        variants.append((label, Path(path).expanduser().resolve()))
    extra = collections.defaultdict(list)
    for text in args.variant_arg:
        label, _, argument = text.partition(":")
        extra[label].append(argument)
    sim_run.clean_up = make_cleanup([workspace for _, workspace in variants])

    records = []
    for index in range(args.runs):
        for label, workspace in variants:
            print(f"run {index + 1}/{args.runs} {label} ({args.render}, {args.world}, "
                  f"{args.profile})", flush=True)
            record = one_run(label, workspace, extra[label], args, index)
            print(json.dumps(record), flush=True)
            records.append(record)

    report = {
        "render": args.render, "world": args.world, "profile": args.profile,
        "numpy": sorted({record["numpy"] for record in records}),
        "cpus": os.cpu_count(), "runs": records, "summary": {}}
    for label, _ in variants:
        mine = [flat_figures(record) for record in records
                if record["variant"] == label and "error" not in record]
        names = sorted({name for figures in mine for name in figures})
        report["summary"][label] = {
            name: summarize(mine, name) for name in names}
        report["summary"][label]["failed_runs"] = sum(
            "error" in record for record in records if record["variant"] == label)
        report["summary"][label]["clock_subscribers"] = sorted({
            name for record in records if record["variant"] == label
            for name in record.get("clock_subscribers", [])})
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
