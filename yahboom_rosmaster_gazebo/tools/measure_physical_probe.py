#!/usr/bin/env python3
r"""
Run physical_rosmaster's contract probe against the simulator and tally its errors.

This is the harness behind #43 step 9, "parity check in CI". Each run is a fresh
simulator launch (empty.world, headless, ``sensor_profile:=physical``, a seeded
cloud timing model), so start-up behavior is part of what is measured. Fifteen
wall seconds after the cloud first appears, as the launch tests start their
probes, it runs ``tools/physical_contract_probe.py`` once on the ROS clock with
``--ros-args -p use_sim_time:=true -p samples:=N -p target:=simulator``, records its
exit code, every error it reports and the ``frame_id`` the simulator's
``/joint_states`` carries, and stops the simulator. It then prints each distinct
error with its count, and the probe's wall time.

Nothing of physical_rosmaster is vendored. The probe is read with ``git show``
at the commit the parity ledger pins (``--commit`` overrides it) from a local
checkout, which is never touched; see ``physical_probe.py``. Physical_rosmaster's
Apache-2.0 header stays on the copy. ``--patch`` applies a patch to the copy, and
``--param name:=value`` adds probe parameters (``target:=simulator`` is the
default).

Run it with ROS sourced, from a source tree, with no simulator running::

    python3 tools/measure_physical_probe.py --workspace ~/Documents/rosmaster_ws \
        --runs 20 --samples 10 --output probe_gpu.json

``--render llvmpipe`` reproduces CI's software rendering pinned to four CPUs.
"""

import argparse
import collections
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

from measure_cloud_timing import wait_for_publisher  # noqa: E402
from physical_probe import fetch_contract_probe, ledger_pin, repository  # noqa: E402
from sim_run import (  # noqa: E402
    isolated_environment,
    LLVMPIPE_ENV,
    ROS_SETUP,
    Simulator,
)

CLOUD = "/cam_1/depth/color/points"
# As the launch tests start their probes, once the sensors are publishing.
DEFAULT_START_DELAY_S = 15.0
LAUNCH_ARGUMENTS = (
    "headless:=true", "rviz:=false", "use_sim_time:=true", "world:=empty.world",
    "sensor_profile:={profile}", "sensor_seed:={seed}")
FAILED = re.compile(r"Physical contract FAILED: (.*)$", re.MULTILINE)
RECEIVED = re.compile(r"Received: (.*)$", re.MULTILINE)
PASSED = re.compile(r"Physical contract PASSED", re.MULTILINE)
# Numbers that vary from run to run are not part of an error's identity.
NUMBER = re.compile(r"-?\d+\.\d+")


def real_time_factor(window_s=5.0):
    """Return sim seconds per wall second over ``window_s`` of /clock, or None."""
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node("measure_physical_probe_clock")
    seen = []
    node.create_subscription(
        Clock, "/clock",
        lambda message: seen.append((
            time.monotonic(), message.clock.sec + message.clock.nanosec * 1e-9)),
        10)
    try:
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(seen) < 2 or seen[-1][0] <= seen[0][0]:
        return None
    return (seen[-1][1] - seen[0][1]) / (seen[-1][0] - seen[0][0])


def error_key(error):
    """Return an error with its varying numbers blanked, so equal faults tally together."""
    return NUMBER.sub("N", error)


def split_errors(output):
    """Return the probe's errors from its log, or an empty list when it passed."""
    found = FAILED.search(output)
    return [part.strip() for part in found.group(1).split("; ")] if found else []


def joint_state_frame(environment):
    """Return the frame_id carried by one /joint_states message, or None."""
    command = (
        f"source {ROS_SETUP} && timeout 20 ros2 topic echo --once "
        "--field header.frame_id /joint_states")
    merged = dict(os.environ)
    merged.update(environment)
    completed = subprocess.run(
        ["bash", "-c", command], env=merged, capture_output=True, text=True)
    lines = [line for line in completed.stdout.splitlines()
             if line and line != "---" and not line.startswith("WARNING")]
    return lines[0].strip("'\"") if lines else None


def joint_state_frame_retrying(environment, attempts=3):
    """Return the /joint_states frame_id; the echo CLI sometimes prints a notice instead."""
    for _ in range(attempts):
        frame = joint_state_frame(environment)
        if frame and "lost" not in frame:
            return frame
    return frame


def run_probe(probe, samples, timeout_s, parameters, environment, log_path):
    """Run the probe once, on the ROS clock, and return (exit code, wall s, output)."""
    merged = dict(os.environ)
    merged.update(environment)
    command = (
        f"source {ROS_SETUP} && exec python3 {probe} --ros-args "
        f"-p use_sim_time:=true -p samples:={samples} -p timeout:={timeout_s:.1f}"
        + "".join(f" -p {parameter}" for parameter in parameters or ()))
    started = time.monotonic()
    try:
        completed = subprocess.run(
            ["bash", "-c", command], env=merged, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, timeout=timeout_s + 60.0)
        code, output = completed.returncode, completed.stdout
    except subprocess.TimeoutExpired as error:
        code, output = "hung", (error.stdout or "")
    Path(log_path).write_text(output, encoding="utf-8")
    return code, time.monotonic() - started, output


def one_run(index, args, probe, work_dir):
    """Launch the simulator, run the probe against it and return the run's record."""
    seed = args.seed_base + index
    domain = 60 + (os.getpid() + index) % 150
    environment = isolated_environment(domain, f"physical_probe_{os.getpid()}_{index}")
    environment["PYTHONNOUSERSITE"] = "1"
    simulator = Simulator(
        Path(args.workspace).expanduser().resolve(),
        work_dir / f"simulator_{index}.log",
        [text.format(profile=args.profile, seed=seed) for text in LAUNCH_ARGUMENTS],
        environment, render=args.render)
    record = {"run": index, "seed": seed}
    try:
        simulator.start()
        os.environ.update(environment)
        if args.render == "llvmpipe":
            os.environ.update(LLVMPIPE_ENV)
        ready_timeout = 240.0 if args.render == "llvmpipe" else 90.0
        if not wait_for_publisher(CLOUD, ready_timeout):
            record.update(code="never ready", errors=["no publisher on " + CLOUD])
            return record
        time.sleep(args.start_delay)
        record["joint_states_frame_id"] = joint_state_frame_retrying(environment)
        factor = real_time_factor()
        record["real_time_factor"] = None if factor is None else round(factor, 3)
        code, wall, output = run_probe(
            probe, args.samples, args.timeout, args.param, environment,
            work_dir / f"probe_{index}.log")
        errors = split_errors(output)
        if code != 0 and not errors:
            # Neither verdict was printed: the probe itself broke.
            last = [line for line in output.splitlines() if line.strip()][-1:]
            errors = ["probe did not finish: " + (last[0] if last else "no output")]
        record.update(code=code, wall_s=round(wall, 1), errors=errors)
        received = RECEIVED.search(output)
        if received:
            record["received"] = received.group(1)
        record["passed"] = bool(PASSED.search(output))
    finally:
        record["shutdown"] = simulator.stop()
    return record


def spread(values):
    """Return n, median and range of ``values``, or None when there are none."""
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    return {"n": len(values), "median": values[len(values) // 2],
            "min": values[0], "max": values[-1]}


def wall_summary(records):
    """Return the spread of the probe's wall time over the runs."""
    return spread(record.get("wall_s") for record in records)


def rtf_summary(records):
    """Return the spread of the real-time factor over the runs."""
    return spread(record.get("real_time_factor") for record in records)


def tally(records):
    """Return the report: runs, passes and each distinct error with its run count."""
    errors = collections.Counter()
    examples = {}
    for record in records:
        for key in {error_key(error) for error in record.get("errors", [])}:
            errors[key] += 1
        for error in record.get("errors", []):
            examples.setdefault(error_key(error), error)
    return {
        "runs": len(records),
        "passed": sum(1 for record in records if record.get("passed")),
        "exit_codes": dict(collections.Counter(str(r.get("code")) for r in records)),
        "shutdown": dict(collections.Counter(r.get("shutdown") for r in records)),
        "probe_wall_s": wall_summary(records),
        "real_time_factor": rtf_summary(records),
        "joint_states_frame_id": dict(
            collections.Counter(str(r.get("joint_states_frame_id")) for r in records)),
        "errors": [
            {"count": count, "error": key, "example": examples[key]}
            for key, count in errors.most_common()],
    }


def main():
    """Run the probe ``--runs`` times, one launch each, and print the tally."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workspace", required=True, help="a built workspace to measure")
    parser.add_argument(
        "--physical-repo",
        help="a local physical_rosmaster checkout, only read; default "
        "$PHYSICAL_ROSMASTER_REPO or ~/Documents/air-club/physical_rosmaster")
    parser.add_argument(
        "--commit", default=ledger_pin(),
        help="physical_rosmaster commit (full SHA); default the ledger's pin")
    parser.add_argument(
        "--patch", action="append",
        help="a patch to apply to the fetched probe, in order; repeatable")
    parser.add_argument(
        "--param", action="append", default=["target:=simulator"],
        help="an extra probe parameter as name:=value, such as target:=simulator; repeatable")
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="the probe's own wall-clock timeout, in seconds")
    parser.add_argument("--start-delay", type=float, default=DEFAULT_START_DELAY_S,
                        help="wall seconds from the cloud's first publisher to the probe")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--profile", choices=("physical", "ideal"), default="physical")
    parser.add_argument("--seed-base", type=int, default=0,
                        help="run i uses sensor_seed seed_base + i, from 1")
    parser.add_argument("--work-dir", default="physical_probe_work")
    parser.add_argument("--output", help="write the JSON report here")
    args = parser.parse_args()

    work_dir = Path(args.work_dir).expanduser().resolve()
    probe, blob = fetch_contract_probe(
        Path(args.physical_repo).expanduser() if args.physical_repo else repository(),
        args.commit, work_dir, args.patch or ())
    records = []
    for index in range(1, args.runs + 1):
        record = one_run(index, args, probe, work_dir)
        records.append(record)
        print(f"run {index}/{args.runs}: code={record.get('code')} "
              f"errors={record.get('errors')}", flush=True)
    report = tally(records)
    report["measurement"] = {
        "physical_commit": args.commit, "patches": args.patch,
        "parameters": args.param, "probe_blob": blob,
        "render": args.render, "profile": args.profile, "samples": args.samples,
        "start_delay_s": args.start_delay,
        "uname": os.uname().release,
    }
    report["records"] = records
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
