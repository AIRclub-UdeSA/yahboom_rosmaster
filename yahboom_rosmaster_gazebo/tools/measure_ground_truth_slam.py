#!/usr/bin/env python3
r"""
Check that ground_truth_base keeps its map alignment while SLAM corrects map -> odom.

ground_truth_tf.py captures one fixed ``map <- world`` alignment when slam_toolbox
publishes ``map -> odom`` and must not move ``ground_truth_base`` with later
corrections. Each run starts ``yahboom_rosmaster_slam slam.launch.py`` headless,
drives the robot with /cmd_vel so that slam_toolbox corrects ``map -> odom``, and
listens to /tf and /ground_truth/odom for a window. It reports:

* the parent frames ``ground_truth_base`` was published in, in order, and how
  many times it changed (``odom`` first, then ``map`` once);
* the ground-truth log lines of the launch ("Captured fixed odom <- world", then
  "Ground-truth display switched from odom to map", which is how the map
  alignment is logged), counted;
* the alignment each ``ground_truth_base`` transform implies,
  ``parent <- ground_truth_base`` composed with the inverse of the
  ``world <- base_footprint`` pose of the odometry message with the same stamp,
  as x, y and yaw. While the display frame is ``map`` it must be constant. Its
  spread (max - min) is reported next to the spread of ``map -> odom`` over the
  same window, which shows the corrections the alignment ignores;
* the rate of ``ground_truth_base`` on stamps and the share of its stamps equal
  to an odometry stamp.

The planar composition below is written out here on purpose: a probe must not
share the math of the node it checks. The robot drives on a flat floor, so
x, y and yaw describe a pose completely.

Run it from a source tree with ROS sourced and no other simulator on the
machine; the cleanup is ``scripts/clean_sim.sh`` over the workspace::

    python3 tools/measure_ground_truth_slam.py --workspace ~/Documents/rosmaster_ws \
        --world cafe.world --output slam_check.json
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import sim_run  # noqa: E402
from sim_run import isolated_environment  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
# Forward, turn, forward again, turn the other way: slam_toolbox sees motion
# and corrects the drift of the wheel odometry.
DRIVE_PATTERN = (
    (0.10, 0.0, 4.0), (0.0, 0.0, 1.0), (0.0, 0.6, 6.0), (0.0, 0.0, 1.0),
    (0.10, 0.0, 4.0), (0.0, 0.0, 1.0), (0.0, -0.6, 6.0), (0.0, 0.0, 1.0))


def yaw_of(rotation):
    """Return the yaw of a quaternion that only rotates about z."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))


def planar(x, y, yaw):
    """Return a planar pose as a tuple."""
    return (x, y, yaw)


def compose(first, second):
    """Return first * second for planar poses."""
    cosine, sine = math.cos(first[2]), math.sin(first[2])
    return (
        first[0] + cosine * second[0] - sine * second[1],
        first[1] + sine * second[0] + cosine * second[1],
        first[2] + second[2])


def invert(pose):
    """Return the inverse of a planar pose."""
    cosine, sine = math.cos(pose[2]), math.sin(pose[2])
    return (
        -(cosine * pose[0] + sine * pose[1]),
        sine * pose[0] - cosine * pose[1],
        -pose[2])


def wrap(angle):
    """Return an angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def spread(values):
    """Return max - min, or None for no values."""
    return max(values) - min(values) if values else None


def stamp_ns(stamp):
    """Return a header stamp in nanoseconds."""
    return stamp.sec * 1000000000 + stamp.nanosec


class Probe:
    """Record /tf and /ground_truth/odom on wall time."""

    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import Twist

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("ground_truth_slam_probe")
        self.truth = {}
        self.tf_truth = []
        self.map_odom = []
        self.window = False
        self.seen_map_odom = False
        self.node.create_subscription(
            Odometry, "/ground_truth/odom", self._on_odom, qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.Twist = Twist

    def _on_odom(self, message):
        # Always recorded: a transform at the window's edge is made from an
        # odometry message that arrived just before the window opened.
        pose = message.pose.pose
        self.truth[stamp_ns(message.header.stamp)] = planar(
            pose.position.x, pose.position.y, yaw_of(pose.orientation))

    def _on_tf(self, message):
        for transform in message.transforms:
            translation = transform.transform.translation
            pose = planar(
                translation.x, translation.y, yaw_of(transform.transform.rotation))
            if transform.child_frame_id == "ground_truth_base" and self.window:
                self.tf_truth.append(
                    (stamp_ns(transform.header.stamp), transform.header.frame_id, pose))
            elif (transform.header.frame_id == "map" and transform.child_frame_id == "odom"):
                self.seen_map_odom = True
                if self.window:
                    self.map_odom.append((stamp_ns(transform.header.stamp), pose))

    def spin_for(self, seconds):
        """Spin for ``seconds`` of wall time."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def drive(self, linear, angular, seconds):
        """Publish one velocity at 20 Hz for ``seconds`` of wall time."""
        deadline = time.monotonic() + seconds
        message = self.Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        while time.monotonic() < deadline:
            self.cmd.publish(message)
            self.spin_for(0.05)

    def shutdown(self):
        """Release the node."""
        self.node.destroy_node()
        self.rclpy.shutdown()


def analyze(probe, log_text):
    """Return the figures of one run from the probe's records and the launch log."""
    frames = []
    for _, parent, _ in probe.tf_truth:
        if not frames or frames[-1] != parent:
            frames.append(parent)
    figures = {
        "ground_truth_parent_frames_in_order": frames,
        "ground_truth_parent_changes": len(frames) - 1,
        "log_captured_odom_world": log_text.count("Captured fixed odom <- world"),
        "log_display_switched_odom_to_map": log_text.count(
            "Ground-truth display switched from odom to map"),
        "log_could_not_align": log_text.count("Could not align ground truth"),
    }
    stamps = [stamp for stamp, _, _ in probe.tf_truth]
    figures["ground_truth_base_count"] = len(stamps)
    if len(stamps) > 1:
        span_s = (stamps[-1] - stamps[0]) * 1e-9
        figures["ground_truth_base_rate_hz_on_stamps"] = (len(stamps) - 1) / span_s
        figures["share_stamped_like_an_odom_message"] = sum(
            stamp in probe.truth for stamp in stamps) / len(stamps)
    for parent in ("odom", "map"):
        offsets = [
            compose(pose, invert(probe.truth[stamp]))
            for stamp, frame, pose in probe.tf_truth
            if frame == parent and stamp in probe.truth]
        figures[f"alignment_samples_in_{parent}"] = len(offsets)
        if offsets:
            figures[f"alignment_in_{parent}_spread"] = {
                "x_m": spread([o[0] for o in offsets]),
                "y_m": spread([o[1] for o in offsets]),
                "yaw_rad": spread([wrap(o[2] - offsets[0][2]) for o in offsets]),
                "first_x_y_yaw": list(offsets[0])}
    map_frame_stamps = [
        stamp for stamp, frame, _ in probe.tf_truth if frame == "map"]
    if map_frame_stamps:
        window = [
            pose for stamp, pose in probe.map_odom
            if map_frame_stamps[0] <= stamp <= map_frame_stamps[-1]]
        figures["map_to_odom_samples_while_in_map"] = len(window)
        if window:
            figures["map_to_odom_spread_while_in_map"] = {
                "x_m": spread([p[0] for p in window]),
                "y_m": spread([p[1] for p in window]),
                "yaw_rad": spread([wrap(p[2] - window[0][2]) for p in window])}
    return figures


def stop(process, timeout_s=40.0):
    """Stop the launch with SIGINT, then kill its session; return how it ended."""
    outcome = "clean"
    launch = [
        pid for pid, (command, _) in sim_run.session_processes(process.pid).items()
        if "ros2" in command and "launch" in command and "bash -c" not in command]
    for pid in launch:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and process.poll() is None:
        time.sleep(0.5)
    if process.poll() is None:
        outcome = "stalled"
    for pid in sim_run.session_processes(process.pid):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return outcome


def one_run(args, index):
    """Run one SLAM launch and return its figures."""
    workspace = Path(args.workspace).expanduser().resolve()
    domain = 60 + (os.getpid() + index) % 150
    environment = isolated_environment(domain, f"gt_slam_{os.getpid()}_{index}")
    if args.no_user_site:
        environment["PYTHONNOUSERSITE"] = "1"
    log = Path(args.output_dir) / f"{args.label}_{index}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["bash", str(REPOSITORY_ROOT / "scripts" / "clean_sim.sh"), str(workspace)],
        check=False)
    merged = dict(os.environ)
    merged.update(environment)
    command = sim_run._ros_shell(
        "exec ros2 launch yahboom_rosmaster_slam slam.launch.py headless:=true "
        f"open_rviz:=false world:={args.world}", workspace)
    with open(log, "w", encoding="utf-8") as handle:
        process = subprocess.Popen(
            ["bash", "-c", command], stdout=handle, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, env=merged, start_new_session=True)
    os.environ.update(environment)
    record = {"label": args.label, "run": index, "workspace": str(workspace)}
    probe = None
    try:
        probe = Probe()
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline and not probe.seen_map_odom:
            probe.spin_for(0.5)
        if not probe.seen_map_odom:
            record["error"] = "no map -> odom within the start-up timeout"
            return record
        probe.spin_for(5.0)
        probe.window = True
        wall_start = time.monotonic()
        while time.monotonic() - wall_start < args.window_s:
            for linear, angular, seconds in DRIVE_PATTERN:
                if time.monotonic() - wall_start >= args.window_s:
                    break
                probe.drive(linear, angular, seconds)
        probe.drive(0.0, 0.0, 1.0)
        probe.window = False
        record["window_wall_s"] = time.monotonic() - wall_start
        probe.spin_for(0.5)
        record["figures_ready"] = True
    finally:
        if probe is not None:
            probe.shutdown()
        record["shutdown"] = stop(process)
    figures = analyze(probe, log.read_text(encoding="utf-8", errors="replace"))
    record.update(figures)
    return record


def main():
    """Run the SLAM check ``--runs`` times and print a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--workspace", required=True, help="a built colcon workspace")
    parser.add_argument("--world", default="cafe.world")
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--window-s", type=float, default=60.0)
    parser.add_argument("--label", default="slam")
    parser.add_argument("--no-user-site", action="store_true")
    parser.add_argument("--output-dir", default="ground_truth_slam_logs")
    parser.add_argument("--output")
    args = parser.parse_args()
    records = []
    for index in range(args.runs):
        print(f"run {index + 1}/{args.runs} {args.label}", flush=True)
        record = one_run(args, index)
        print(json.dumps(record), flush=True)
        records.append(record)
    text = json.dumps({"runs": records}, indent=2)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
