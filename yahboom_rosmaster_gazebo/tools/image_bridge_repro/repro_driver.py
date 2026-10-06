#!/usr/bin/env python3
r"""
Run the image_bridge shutdown race without Gazebo (#55).

Each trial starts ``gz_image_publisher`` (two image topics on Gazebo transport,
standing in for the camera) and one image_bridge build, lets them connect, then
sends the bridge SIGINT and records its exit status. ``--pause`` first tells the
publisher to stop publishing (as a paused world does) and waits ``--settle``
seconds. One JSON line per trial goes to ``--out``; counts are printed at the end.

    source /opt/ros/humble/setup.bash
    cmake -S tools/image_bridge_repro -B /tmp/ibr && cmake --build /tmp/ibr
    python3 tools/image_bridge_repro/repro_driver.py --build /tmp/ibr \
        --variant orig --rate 30 --trials 200 --out /tmp/ibr_orig.jsonl

``--trace`` preloads tools/segv_trace's library so a crash leaves a backtrace.
Run it with no simulator running: it uses its own transport partition.
"""

import argparse
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import time

CONNECT_SECONDS = 2.5


def trial(args, number):
    """Run one trial and return its record."""
    env = dict(os.environ)
    env["ROS_LOCALHOST_ONLY"] = "1"
    env["ROS_DOMAIN_ID"] = str(args.domain)
    env["IGN_PARTITION"] = f"ibr_{os.getpid()}_{number}"
    env["OMP_NUM_THREADS"] = "1"
    if args.trace:
        env["LD_PRELOAD"] = str(args.trace)
        env["SEGV_TRACE_COMMS"] = f"image_bridge_{args.variant}"[:15]  # comm is 15 chars
        env["SEGV_TRACE_DIR"] = str(args.out.parent)
    publisher = subprocess.Popen(
        [str(args.build / "gz_image_publisher"), str(args.rate), str(args.width),
         str(args.height), "/cam_1/image", "/cam_1/depth_image"],
        env=env, start_new_session=True)
    bridge = subprocess.Popen(
        [str(args.build / f"image_bridge_{args.variant}"),
         "/cam_1/image", "/cam_1/depth_image",
         "--ros-args", "-r", "/cam_1/image:=/internal/cam_1/color/image_raw",
         "-r", "/cam_1/depth_image:=/internal/cam_1/depth/image_raw"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)
    time.sleep(CONNECT_SECONDS + random.random() * 0.2)
    if args.pause:
        publisher.send_signal(signal.SIGUSR1)
        time.sleep(args.settle)
    start = time.monotonic()
    bridge.send_signal(signal.SIGINT)
    try:
        code = bridge.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(bridge.pid, signal.SIGKILL)
        code = "hung"
    elapsed = time.monotonic() - start
    for process in (publisher, bridge):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
    return {"trial": number, "variant": args.variant, "rate": args.rate,
            "pause": args.pause, "exit": code, "stop_s": round(elapsed, 3)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--variant", choices=("orig", "guard"), default="orig")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--pause", action="store_true")
    parser.add_argument("--settle", type=float, default=0.25)
    parser.add_argument("--trace", type=Path)
    parser.add_argument("--domain", type=int, default=88)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    counts = {}
    with open(args.out, "a") as handle:
        for number in range(args.trials):
            record = trial(args, number)
            handle.write(json.dumps(record) + "\n")
            handle.flush()
            counts[record["exit"]] = counts.get(record["exit"], 0) + 1
    print(f"{args.variant} rate={args.rate} pause={args.pause}: "
          f"n={args.trials} exits={counts}")


if __name__ == "__main__":
    main()
