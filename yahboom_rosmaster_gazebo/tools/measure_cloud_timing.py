#!/usr/bin/env python3
r"""
Measure the simulator's camera topics with physical_rosmaster's own probe.

This is the harness behind the ``step_measurements`` records of the parity
ledger (config/real_robot_contract.yaml): it runs the physical robot's
``tools/sensor_capability_probe.py`` against the simulator, as
``--group camera --duration 40 --camera-frame-rate 30.30303`` (the simulated
camera's stamps come 33 ms apart, 30.30303 Hz), three times in one launch, on
empty.world, headless, under ``sensor_profile:=physical`` with a random seed.
It reports, pooled over the runs and on sim time:

* the point cloud's rate, stamp period p95, latency, the histogram of frames from
  one cloud to the next, its median, p95, longest and mean, and the worst gap;
* the depth image's rate and latency;
* the depth image's valid and NaN fractions, with the scene they were seen in.

Nothing of physical_rosmaster is vendored. The probe and the contract probe it
imports are fetched from a pinned commit of a local checkout with ``git show``,
never checked out, into a work directory; then a small committed patch
(``patches/sensor_capability_probe_sim_time.patch``) is applied to the probe
copy: it runs the node on ``use_sim_time`` and stamps receipt with the ROS clock,
so that ``latency_ms`` is sim time at receipt minus ``header.stamp``, as the
step-2 baseline of #43 requires of any simulator run. Without it the probe
reads wall epoch minus sim time. The copy keeps physical_rosmaster's Apache-2.0
header, since it is theirs; the patch is ours and touches three lines.

Run it with ROS sourced, from a source tree, with no simulator running::

    python3 tools/measure_cloud_timing.py --workspace ~/Documents/rosmaster_ws_step7 \
        --physical-repo ~/Documents/air-club/physical_rosmaster \
        --output step7_camera.json

It starts, watches and stops the simulator itself (see ``sim_run.py``). On
llvmpipe pass ``--render llvmpipe``; the rates then read low on wall time but the
stamp-based figures hold.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import statistics
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from cloud_timing import gap_statistics  # noqa: E402
from sim_run import ROS_SETUP, Simulator, isolated_environment  # noqa: E402

# physical_rosmaster main after #44 and #45 merged (#45's merge commit).
PHYSICAL_PIN = "468662ca25a52515a218dd944fc031ca85266244"
# The probe imports the contract probe, so the two are fetched side by side.
PROBE_FILES = ("tools/sensor_capability_probe.py", "tools/physical_contract_probe.py")
PATCH = Path(__file__).resolve().parent / "patches" / "sensor_capability_probe_sim_time.patch"
FRAME_RATE_HZ = 30.30303
CLOUD = "/cam_1/depth/color/points"
DEPTH = "/cam_1/depth/image_raw"
LAUNCH_ARGUMENTS = (
    "headless:=true", "rviz:=false", "use_sim_time:=true", "world:={world}",
    "sensor_profile:={profile}", "sensor_seed:=-1", "motion_bias:=false")
SETTLE_S = 15.0


def fetch_probe(repository, commit, work_dir):
    """
    Write the probe files of ``commit`` into ``work_dir``, patched, and return the probe's path.

    Reads with ``git show``, so the checkout's working tree and branch are never
    touched. The patch must apply cleanly: if physical_rosmaster changes the
    lines it edits, this fails instead of measuring something else.
    """
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError(f"commit must be a full 40-character SHA, got {commit!r}")
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    for name in PROBE_FILES:
        text = subprocess.run(
            ["git", "-C", str(repository), "show", f"{commit}:{name}"],
            check=True, capture_output=True, text=True).stdout
        (work_dir / Path(name).name).write_text(text, encoding="utf-8")
    probe = work_dir / "sensor_capability_probe.py"
    subprocess.run(
        ["patch", "-p1", "--forward", "--no-backup-if-mismatch", "-i", str(PATCH)],
        cwd=work_dir, check=True, capture_output=True, text=True)
    return probe


def wait_for_publisher(topic, timeout_s):
    """Return whether ``topic`` has a publisher within ``timeout_s`` wall seconds."""
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("measure_cloud_timing_wait")
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.count_publishers(topic):
                return True
        return False
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_probe(probe, duration_s, output, environment):
    """Run the probe once, on the camera group, and return its parsed JSON."""
    merged = dict(os.environ)
    merged.update(environment)
    command = (
        f"source {ROS_SETUP} && exec python3 {probe} --group camera "
        f"--duration {duration_s:g} --camera-frame-rate {FRAME_RATE_HZ} "
        f"--per-message --output {output}")
    subprocess.run(
        ["bash", "-c", command], env=merged, check=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    return json.loads(Path(output).read_text(encoding="utf-8"))


def topic_summary(result, topic):
    """Return the summary of ``topic`` in one probe result."""
    for summary in result["topics"]:
        if summary["topic"] == topic:
            return summary
    raise KeyError(topic)


def percentile(values, fraction):
    """Return the ``fraction`` quantile of ``values``, the nearest rank at or above it."""
    ordered = sorted(values)
    return ordered[min(len(ordered), max(1, math.ceil(fraction * len(ordered)))) - 1]


def stamp_gaps_s(summary):
    """Return the gaps between consecutive header stamps of one topic, in seconds."""
    stamps = [s for s in summary["per_message"]["stamp_s"] if s is not None]
    return [later - earlier for earlier, later in zip(stamps, stamps[1:])]


def pooled(results):
    """Pool the camera group's figures over several probe results."""
    report = {}
    cloud = [topic_summary(result, CLOUD) for result in results]
    histogram = {}
    for summary in cloud:
        for frames, count in summary["frame_cadence"]["frames_per_gap"].items():
            histogram[int(frames)] = histogram.get(int(frames), 0) + count
    gaps = [gap for summary in cloud for gap in stamp_gaps_s(summary)]
    latencies = [
        value for summary in cloud
        for value in summary["per_message"]["latency_ms"] if value is not None]
    statistics_frames = gap_statistics(histogram)
    period = 1.0 / FRAME_RATE_HZ
    report[CLOUD] = {
        "runs": len(cloud),
        "gaps": sum(histogram.values()),
        "frames_per_gap": {str(k): histogram[k] for k in sorted(histogram)},
        "gap_frames": {
            "median": statistics_frames["median"],
            "p95": statistics_frames["p95"],
            "longest": statistics_frames["longest"],
            "mean": round(statistics_frames["mean"], 3),
        },
        "off_grid_gaps": sum(s["frame_cadence"]["off_grid_gaps"] for s in cloud),
        "rate_hz_pooled": round(1.0 / (statistics_frames["mean"] * period), 3),
        "rate_hz_per_run_wall": [round(s["rate_hz"], 3) for s in cloud],
        "period_p95_ms": round(1000.0 * percentile(gaps, 0.95), 1),
        "worst_gap_s": round(max(gaps), 3),
        "latency_ms_median": round(statistics.median(latencies), 2),
        "latency_ms_p95": round(percentile(latencies, 0.95), 2),
        "latency_ms_run_medians": [round(s["latency_ms"]["median"], 2) for s in cloud],
    }
    depth = [topic_summary(result, DEPTH) for result in results]
    depth_gaps = [gap for summary in depth for gap in stamp_gaps_s(summary)]
    depth_latency = [
        value for summary in depth
        for value in summary["per_message"]["latency_ms"] if value is not None]
    content = [
        item["depth"] for summary in depth
        for item in (summary["content"], summary.get("content_last")) if item]
    report[DEPTH] = {
        "rate_hz_per_run_wall": [round(s["rate_hz"], 3) for s in depth],
        "rate_hz_on_stamps": round(len(depth_gaps) / sum(depth_gaps), 3),
        "latency_ms_median": round(statistics.median(depth_latency), 2),
        "latency_ms_p95": round(percentile(depth_latency, 0.95), 2),
        "valid_fraction": round(statistics.mean(c["valid_fraction"] for c in content), 4),
        "nan_fraction": round(statistics.mean(c["nan_fraction"] for c in content), 4),
        "valid_fraction_range": [
            round(min(c["valid_fraction"] for c in content), 4),
            round(max(c["valid_fraction"] for c in content), 4)],
        "content_samples": len(content),
    }
    return report


def measure(args):
    """Launch the simulator, run the probe ``args.runs`` times and return the report."""
    work_dir = Path(args.work_dir).expanduser().resolve()
    probe = fetch_probe(Path(args.physical_repo).expanduser(), args.commit, work_dir)
    domain = 60 + os.getpid() % 150
    environment = isolated_environment(domain, f"cloud_timing_{os.getpid()}")
    if args.no_user_site:
        environment["PYTHONNOUSERSITE"] = "1"
    os.environ.update(environment)
    simulator = Simulator(
        Path(args.workspace).expanduser().resolve(), work_dir / "simulator.log",
        [text.format(world=args.world, profile=args.profile) for text in LAUNCH_ARGUMENTS],
        environment, render=args.render)
    results, outcome = [], "not started"
    try:
        simulator.start()
        if not wait_for_publisher(CLOUD, 240.0 if args.render == "llvmpipe" else 90.0):
            raise RuntimeError(f"no publisher on {CLOUD}; see {work_dir / 'simulator.log'}")
        time.sleep(SETTLE_S)
        for index in range(args.runs):
            print(f"probe run {index + 1}/{args.runs}", flush=True)
            results.append(run_probe(
                probe, args.duration, work_dir / f"probe_run_{index + 1}.json", environment))
    finally:
        outcome = simulator.stop()
    report = pooled(results)
    report["measurement"] = {
        "physical_commit": args.commit,
        "probe_blob": results[0]["measurement"].get("probe_git_blob"),
        "world": args.world,
        "profile": args.profile,
        "render": args.render,
        "runs": args.runs,
        "duration_s": args.duration,
        "shutdown": outcome,
    }
    return report


def main():
    """Parse the arguments, measure, and print the JSON report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workspace", required=True, help="a built workspace to measure")
    parser.add_argument(
        "--physical-repo", default="~/Documents/air-club/physical_rosmaster",
        help="a local physical_rosmaster checkout; it is only read")
    parser.add_argument("--commit", default=PHYSICAL_PIN, help="physical_rosmaster commit")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--profile", choices=("physical", "ideal"), default="physical")
    parser.add_argument("--world", default="empty.world")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--no-user-site", action="store_true",
                        help="hide ~/.local packages: use the distribution's numpy")
    parser.add_argument("--work-dir", default="cloud_timing_work")
    parser.add_argument("--output", help="write the JSON report here")
    args = parser.parse_args()
    report = measure(args)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
