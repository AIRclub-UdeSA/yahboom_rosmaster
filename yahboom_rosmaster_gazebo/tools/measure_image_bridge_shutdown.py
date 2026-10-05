#!/usr/bin/env python3
"""
Loop a simulator launch test and record how each process left (issue #55).

Each run clears leftovers (``scripts/clean_sim.sh``), launches one launch test
detached in its own session, and keeps its whole log. From the log it records
the exit code of every launched process, the shutdown lines of the launch's
OnShutdown handler with their wall-clock times, and any backtrace written by
``segv_trace/segv_trace.c``. One JSON object per run is appended to
``<out>/runs.jsonl``; ``--summary`` prints counts and rates from it.

    source /opt/ros/humble/setup.bash
    source <workspace>/install/setup.bash
    python3 tools/measure_image_bridge_shutdown.py --workspace <workspace> \\
        --out ~/Documents/rosmaster_ws/measurement_logs/55_image_bridge/gpu_base \\
        --runs 150 [--trace] [--render llvmpipe --cpus 0-3]
    python3 tools/measure_image_bridge_shutdown.py --summary <out> [<out> ...]

``--trace`` preloads ``libsegv_trace.so`` (built here from segv_trace.c) into
every launched process; it only acts in processes named by ``--trace-comms``
(default image_bridge), and attaches gdb from the fault handler, so it does not
change how SIGINT reaches the bridge. ``--render llvmpipe`` reproduces the CI
gate's software rendering under ``xvfb-run`` pinned with ``--cpus``.
Run only one Gazebo on the machine (AGENTS.md); results are per-run so the
loop can be stopped and resumed (``--start`` skips finished run numbers).
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
TEST_LAUNCH = REPO / "yahboom_rosmaster_gazebo" / "test" / "world_smoke.launch.py"
RUN_TIMEOUT = 100.0  # CTest's own limit is 70 s; a hang is recorded either way
XVFB_ARGS = "-screen 0 1280x1024x24 -nolisten tcp"
# Every log line is stored as "<epoch seconds>\t<line>", stamped when the
# harness reads it, because the launch's own INFO lines carry no time.
DIED = re.compile(
    r"^(\S+)\t\[INFO\] \[(\S+)\]: process has died \[pid (\d+), exit code (-?\d+)")
FINISHED = re.compile(
    r"^(\S+)\t\[INFO\] \[(\S+)\]: process has finished cleanly \[pid (\d+)\]")
SENT_SIGINT = re.compile(r"^(\S+)\t\[INFO\] \[(\S+)\]: sending signal 'SIGINT'")
NODE_SIGINT = re.compile(r"^(\S+)\t\[(\S+)\] \[INFO\] \[(\d+\.\d+)\] \[rclcpp\]: signal_handler")
SHUTDOWN_LINES = (
    ("image_bridge_sigint_sent", r"Stopping the image bridge"),
    ("image_bridge_stopped", r"image bridge stopped before"),
    ("image_bridge_stall", r"image bridge did not stop within"),
    ("parameter_bridge_sigint_sent", r"Stopping the parameter bridge"),
    ("parameter_bridge_stopped", r"parameter bridge stopped before"),
    ("parameter_bridge_stall", r"parameter bridge did not stop within"),
    ("paused", r"Paused Gazebo before stopping it"),
    ("pause_unacked", r"did not acknowledge the pause"),
    ("stop_acked", r"Gazebo acknowledged the clean stop request"),
    ("gazebo_clean_stop", r"Gazebo completed its clean stop"),
    ("gazebo_force_kill", r"force-killing it"),
    ("gazebo_server_shutting_down", r"Shutting down ign-gazebo-server"),
    ("probe_finished", r"world_smoke_probe.py-\d+\]: process has finished"),
)


def build_preload(workdir):
    """Compile libsegv_trace.so into workdir and return its path."""
    library = Path(workdir) / "libsegv_trace.so"
    source = HERE / "segv_trace" / "segv_trace.c"
    subprocess.run(
        ["gcc", "-shared", "-fPIC", "-O1", "-o", str(library), str(source)],
        check=True)
    return library


def clean(workspace):
    """Run the repository's cleanup script for this workspace."""
    result = subprocess.run(
        ["bash", str(REPO / "scripts" / "clean_sim.sh"), str(workspace)],
        capture_output=True, text=True, check=False)
    return result.stdout.strip().splitlines()[-1:] or ["?"]


def parse_log(text):
    """Return exit codes and shutdown-phase times (epoch s) from one log."""
    exits = {}
    times = {}
    sigint_sent = {}
    node_sigint = {}
    for line in text.splitlines():
        died = DIED.search(line)
        if died:
            exits[died.group(2)] = {
                "t": float(died.group(1)), "code": int(died.group(4))}
            continue
        finished = FINISHED.search(line)
        if finished:
            exits[finished.group(2)] = {"t": float(finished.group(1)), "code": 0}
            continue
        sent = SENT_SIGINT.search(line)
        if sent:
            sigint_sent.setdefault(sent.group(2), float(sent.group(1)))
            continue
        node = NODE_SIGINT.search(line)
        if node:
            node_sigint.setdefault(node.group(2), float(node.group(3)))
            continue
        for key, pattern in SHUTDOWN_LINES:
            if key not in times and re.search(pattern, line):
                times[key] = float(line.split("\t", 1)[0])
    bad = {name: info for name, info in exits.items() if info["code"] != 0}
    return {
        "exits": exits, "nonzero": bad, "shutdown_times": times,
        "launch_sigint_sent": sigint_sent, "node_sigint_logged": node_sigint}


def one_run(args, number, out, preload):
    """Run one launch test and return its result record."""
    run_dir = out / f"run{number:04d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    clean_state = clean(args.workspace)
    env = dict(os.environ)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_DOMAIN_ID"] = str(args.domain)
    env["OMP_NUM_THREADS"] = "1"
    if args.trace:
        env["LD_PRELOAD"] = str(preload)
        env["SEGV_TRACE_COMMS"] = args.trace_comms
        env["SEGV_TRACE_DIR"] = str(run_dir)
    command = [
        sys.executable, "-u", "-m", "launch_testing.launch_test",
        str(args.test), f"world:={args.world}"]
    if args.render == "llvmpipe":
        env.update(
            LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
            LP_NUM_THREADS="4")
        command = ["xvfb-run", "--auto-servernum", f"--server-args={XVFB_ARGS}"] + command
    if args.cpus:
        command = ["taskset", "-c", args.cpus] + command
    log_path = run_dir / "launch.log"
    started = time.time()
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
        start_new_session=True, cwd=args.workspace)
    log = open(log_path, "w")

    def stamp_lines():
        for raw in process.stdout:
            log.write(f"{time.time():.6f}\t{raw.decode(errors='replace')}")

    reader = threading.Thread(target=stamp_lines, daemon=True)
    reader.start()
    try:
        code = process.wait(timeout=args.timeout)
        hung = False
    except subprocess.TimeoutExpired:
        hung = True
        os.killpg(process.pid, signal.SIGKILL)
        code = process.wait()
    try:  # whatever is left of this session (xvfb-run's Xvfb)
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    reader.join(timeout=5)
    log.close()
    text = log_path.read_text(errors="replace")
    record = {
        "run": number, "variant": args.variant, "test_exit": code, "hung": hung,
        "wall_s": round(time.time() - started, 2), "clean": clean_state,
        "git": args.git, "passed": code == 0,
    }
    record.update(parse_log(text))
    record["backtraces"] = sorted(p.name for p in run_dir.glob("*.bt"))
    record["failed_asserts"] = re.findall(r"AssertionError: (.*)", text)[:3]
    # keep only the log of interesting runs; the rest is a few MB each
    interesting = (
        record["nonzero"] or record["backtraces"] or hung or code != 0
        or "image bridge did not stop" in text)
    if not interesting and not args.keep_logs:
        shutil.rmtree(run_dir)
    return record


def summarize(paths):
    """Print counts and rates for the given output directories."""
    for path in paths:
        runs = [json.loads(line) for line in open(Path(path) / "runs.jsonl")]
        signatures = {}
        for run in runs:
            for name, info in run["nonzero"].items():
                signatures[name] = signatures.get(name, 0) + 1
        stalls = sum("image_bridge_stall" in run["shutdown_times"] for run in runs)
        failed = sum(1 for run in runs if run["test_exit"] != 0)
        print(f"{path}: n={len(runs)} failed_tests={failed} "
              f"image_bridge_stall_logged={stalls} backtraces="
              f"{sum(bool(run['backtraces']) for run in runs)}")
        for name, count in sorted(signatures.items()):
            print(f"    {name}: {count}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--runs", type=int, default=150)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--variant", default="base")
    parser.add_argument("--world", default="laberinto_simple.world")
    parser.add_argument("--test", type=Path, default=TEST_LAUNCH)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument("--trace-comms", default="image_bridge")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--cpus", default="")
    parser.add_argument("--domain", type=int, default=77)
    parser.add_argument("--timeout", type=float, default=RUN_TIMEOUT)
    parser.add_argument("--keep-logs", action="store_true")
    parser.add_argument("--summary", nargs="+")
    args = parser.parse_args()
    if args.summary:
        summarize(args.summary)
        return
    args.git = subprocess.run(
        ["git", "-C", str(REPO), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False).stdout.strip()
    args.out.mkdir(parents=True, exist_ok=True)
    preload = build_preload(args.out) if args.trace else None
    for number in range(args.start, args.start + args.runs):
        record = one_run(args, number, args.out, preload)
        with open(args.out / "runs.jsonl", "a") as handle:
            handle.write(json.dumps(record) + "\n")
        flag = ("NONZERO " + ",".join(record["nonzero"])) if record["nonzero"] else ""
        print(f"run {number} exit={record['test_exit']} wall={record['wall_s']} {flag}",
              flush=True)


if __name__ == "__main__":
    main()
