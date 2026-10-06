#!/usr/bin/env python3
r"""
Check what the bridges still forward after the launch pauses the world (#55).

Starts a headless simulator (``scripts/clean_sim.sh`` first), subscribes raw to
what ``image_bridge`` and ``parameter_bridge`` publish, then pauses the world
with the same ``ign service`` call the launch's ``_pause_gazebo`` makes and
counts messages per topic in four windows:

  before    the second before the pause request
  settle    from the pause acknowledgement until GAZEBO_PAUSE_SETTLE has passed
  after     the following second, when the launch would signal the bridges
  (the pause call itself is timed too)

    source /opt/ros/humble/setup.bash && source <workspace>/install/setup.bash
    python3 tools/probe_pause_delivery.py --workspace <workspace> --runs 5 \
        [--render llvmpipe --cpus 0-3] --out pause_delivery.jsonl

One JSON object per run. A topic that still has messages in "after" is still
forwarded while paused. Run with no other simulator on the machine.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

REPO = Path(__file__).resolve().parents[2]
XVFB_ARGS = "-screen 0 1280x1024x24 -nolisten tcp"
SETTLE = 0.5  # GAZEBO_PAUSE_SETTLE in the launch file
TOPICS = {
    "image_bridge": {
        "/internal/cam_1/color/image_raw": Image,
        "/internal/cam_1/depth/image_raw": Image,
    },
    "parameter_bridge": {
        "/internal/cam_1/color/camera_info": CameraInfo,
        "/internal/scan": LaserScan,
        "/internal/imu/data_raw": Imu,
        "/clock": Clock,
        "/ground_truth/odom": Odometry,
    },
}


def start_sim(args, domain, partition):
    """Launch the simulator detached and return the Popen."""
    subprocess.run(
        ["bash", str(REPO / "scripts" / "clean_sim.sh"), str(args.workspace)],
        check=False, capture_output=True)
    env = dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(domain),
               IGN_PARTITION=partition, OMP_NUM_THREADS="1")
    command = [
        "ros2", "launch", "yahboom_rosmaster_gazebo",
        "rosmaster_gazebo_fortress.launch.py", "headless:=true", "rviz:=false",
        "motion_bias:=false", f"world:={args.world}"]
    if args.render == "llvmpipe":
        env.update(LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                   LP_NUM_THREADS="4")
        command = ["xvfb-run", "--auto-servernum",
                   f"--server-args={XVFB_ARGS}"] + command
    if args.cpus:
        command = ["taskset", "-c", args.cpus] + command
    return subprocess.Popen(
        command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True), env


def stop_sim(process):
    """SIGINT the launch's group, then kill what is left of the session."""
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=25)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def pause_world(env, world_name):
    """Make the call _pause_gazebo makes; return (acknowledged, seconds)."""
    started = time.monotonic()
    result = subprocess.run(
        [shutil.which("ign"), "service", "-s", f"/world/{world_name}/control",
         "--reqtype", "ignition.msgs.WorldControl",
         "--reptype", "ignition.msgs.Boolean", "--timeout", "1500",
         "--req", "pause: true"],
        capture_output=True, text=True, env=env, check=False)
    return "data: true" in result.stdout, time.monotonic() - started


def one_run(args, number):
    """Run the simulator once and return the per-window counts."""
    domain = 120 + number
    process, env = start_sim(args, domain, f"pause_probe_{os.getpid()}_{number}")
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(domain))
    rclpy.init()
    node = rclpy.create_node("pause_probe")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    stamps = {}
    for bridge, topics in TOPICS.items():
        for topic, msg_type in topics.items():
            stamps[topic] = []
            node.create_subscription(
                msg_type, topic,
                lambda _msg, t=topic: stamps[t].append(time.monotonic()),
                qos, raw=True)

    # Spin in a thread so messages are stamped when they arrive, also while
    # the (blocking) pause call is running.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def spin(seconds):
        time.sleep(seconds)

    record = {"run": number, "render": args.render, "world": args.world}
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not all(
                len(stamps[t]) > 5 for t in stamps if t != "/clock"):
            time.sleep(0.05)
        spin(args.warmup)
        ready = all(len(stamps[t]) > 5 for t in stamps if t != "/clock")
        record["ready"] = ready
        if ready:
            t_request = time.monotonic()
            acknowledged, call_seconds = pause_world(env, args.world_name)
            t_ack = time.monotonic()
            spin(SETTLE + 1.5)
            record.update(acknowledged=acknowledged,
                          pause_call_s=round(call_seconds, 3))
            windows = {
                "before": (t_request - 1.0, t_request),
                "during_call": (t_request, t_ack),
                "settle": (t_ack, t_ack + SETTLE),
                "after": (t_ack + SETTLE, t_ack + SETTLE + 1.0),
            }
            record["counts"] = {
                topic: {name: sum(lo <= s < hi for s in series)
                        for name, (lo, hi) in windows.items()}
                for topic, series in stamps.items()}
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        stop_sim(process)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--world", default="laberinto_simple.world")
    parser.add_argument("--world-name", default=None,
                        help="SDF world name; read from the world file if omitted")
    parser.add_argument("--render", choices=("gpu", "llvmpipe"), default="gpu")
    parser.add_argument("--cpus", default="")
    parser.add_argument("--warmup", type=float, default=3.0)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    share = args.workspace / "install" / "yahboom_rosmaster_gazebo" / "share" / \
        "yahboom_rosmaster_gazebo" / "worlds" / args.world
    args.world = str(share)
    if args.world_name is None:
        import xml.etree.ElementTree as ET
        args.world_name = ET.parse(share).getroot().find("world").get("name")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    for number in range(args.runs):
        record = one_run(args, number)
        with open(args.out, "a") as handle:
            handle.write(json.dumps(record) + "\n")
        print(json.dumps(record), flush=True)


if __name__ == "__main__":
    sys.exit(main())
