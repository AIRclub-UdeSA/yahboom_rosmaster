#!/usr/bin/env python3
"""
Grade the simulated point cloud's timing, on sim time only.

The gaps between cloud stamps, in whole camera frames, are compared with the
sensor profile's measured distribution; the latency is the sim time a cloud is
received at, minus its stamp. Nothing here reads the wall clock, so the grades
hold under software rendering, where every sim second takes several wall ones.
"""

import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from cloud_timing import (
    cumulative_within,
    normalized,
    quantile_interval,
)
from sensor_profiles import default_path, load_sensor_profile


CLOUD_TOPIC = "/cam_1/depth/color/points"
# The frames' stamps come from camera_info, one small message per frame, not
# from the images: receiving 30 images a second starved the probe under
# software rendering and lost the clouds it was there to count.
FRAME_TOPIC = "/cam_1/color/camera_info"
DEPTH_TOPIC = "/cam_1/depth/image_raw"
CLOUD_FRAME = "cam_1_depth_frame"
# A stamp gap is a whole number of frames to within this, in seconds. The
# simulated camera's stamps are not on an exact grid: its frames come 32, 33
# or 34 ms apart, and over many frames they wander up to 2 ms from a
# 33 ms grid (7 of 122 gaps did so under software rendering). Half a frame
# would still catch a cloud stamped at any time but its frame's, so this
# stays a tenth of one.
OFF_GRID_TOLERANCE_S = 3e-3
FLOAT_SLACK_S = 1e-9
# A cloud carries the exact stamp of the frame it was built from. A few may
# match no image received, since a Best Effort image can be lost in transit.
UNMATCHED_STAMP_FRACTION = 0.02
# Gazebo's camera catches up on the frames it owes when it starts, stamped a
# few milliseconds apart. A frame closer than this fraction of the frame
# period to its predecessor belongs to that burst, and the clouds up to the
# last of them are not graded.
BURST_FRACTION = 0.75
# The median latency may sit this far below and above the profile's. A sim
# clock tick and the probe's own callback account for the difference above.
LATENCY_BELOW_S = 0.002
LATENCY_ABOVE_S = 0.008
# A cloud this much later than the profile's latency counts as late.
LATE_MARGIN_S = 0.003
# The ideal profile adds no latency: a cloud may lag the depth image built from
# the same frame by this much.
IDEAL_LAG_S = 0.003
IDEAL_DELIVERED_FRACTION = 0.97
IDEAL_LONGEST_GAP = 3
DEVIATIONS = 4.0
# No gap may exceed the longest in the table by more than this many frames.
LONGEST_GAP_MARGIN = 6
MINIMUM_GAPS = 30


def layout_errors(cloud):
    """Return the ways a cloud message departs from the physical adapter's layout."""
    errors = []
    fields = [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields]
    expected = [
        ("x", 0, PointField.FLOAT32, 1),
        ("y", 4, PointField.FLOAT32, 1),
        ("z", 8, PointField.FLOAT32, 1),
        ("rgb", 12, PointField.FLOAT32, 1),
    ]
    if fields != expected:
        errors.append(f"fields {fields}, expected {expected}")
    if cloud.point_step != 16:
        errors.append(f"point_step {cloud.point_step}, expected 16")
    if cloud.height != 1:
        errors.append(f"height {cloud.height}, expected an unorganized cloud")
    if not cloud.is_dense:
        errors.append("is_dense is false, expected true with the NaN points stripped")
    if cloud.is_bigendian:
        errors.append("is_bigendian is true")
    if cloud.header.frame_id != CLOUD_FRAME:
        errors.append(f"frame {cloud.header.frame_id!r}, expected {CLOUD_FRAME!r}")
    if cloud.row_step != cloud.point_step * cloud.width:
        errors.append("row_step is not point_step * width")
    if len(cloud.data) != cloud.row_step * cloud.height:
        errors.append("data length is not row_step * height")
    return errors


def percentile(values, fraction):
    """Return the value at a fraction of a non-empty list, by nearest rank."""
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def grade_timing(
        cloud_stamps_ns, cloud_latencies_s, image_stamps_ns, depth_stamps_ns,
        depth_latencies_s, profile_name, profile):
    """
    Return (errors, statistics) for the clouds a probe received.

    ``cloud_stamps_ns`` and ``cloud_latencies_s`` are per cloud, in arrival
    order. ``image_stamps_ns`` and ``depth_stamps_ns`` are the color and depth
    frames' stamps, from which the simulated camera's own frame period is
    measured, and ``depth_latencies_s`` the latency of the public depth
    images, the ideal profile's reference.
    """
    errors = []
    frames_seen = set(image_stamps_ns) | set(depth_stamps_ns)
    ordered = sorted(frames_seen)
    if len(ordered) >= 3:
        typical = statistics.median(
            later - earlier for earlier, later in zip(ordered, ordered[1:]))
        burst = [later for earlier, later in zip(ordered, ordered[1:])
                 if later - earlier < BURST_FRACTION * typical]
    else:
        burst = []
    skipped = 0
    if burst:
        settled = max(burst)
        keep = [index for index, stamp in enumerate(cloud_stamps_ns) if stamp > settled]
        skipped = len(cloud_stamps_ns) - len(keep)
        cloud_stamps_ns = [cloud_stamps_ns[index] for index in keep]
        cloud_latencies_s = [cloud_latencies_s[index] for index in keep]
    stats = {"clouds": len(cloud_stamps_ns), "startup_burst_clouds_skipped": skipped}
    # A cloud from before the first frame's stamp arrived can not be matched:
    # the probe's subscriptions do not all connect at the same moment.
    comparable = [stamp for stamp in cloud_stamps_ns if ordered and stamp >= ordered[0]]
    unmatched = sum(1 for stamp in comparable if stamp not in frames_seen)
    stats["stamps_matching_no_frame"] = unmatched
    stats["clouds_before_first_frame"] = len(cloud_stamps_ns) - len(comparable)
    if unmatched > UNMATCHED_STAMP_FRACTION * len(comparable):
        errors.append(
            f"{unmatched} of {len(comparable)} cloud stamps match no camera frame "
            "received, but a cloud keeps the stamp of the frame it was built from")
    images = sorted(set(image_stamps_ns))
    if len(images) < 3 or len(cloud_stamps_ns) < 2:
        return [f"too few messages: {len(images)} images, {len(cloud_stamps_ns)} clouds"], stats
    period = statistics.median(
        (later - earlier) * 1e-9 for earlier, later in zip(images, images[1:]))
    stats["frame_period_ms"] = round(period * 1000.0, 3)

    gaps_s = [(later - earlier) * 1e-9
              for earlier, later in zip(cloud_stamps_ns, cloud_stamps_ns[1:])]
    if any(gap <= 0.0 for gap in gaps_s):
        return ["cloud stamps do not strictly increase"], stats
    frames = [gap / period for gap in gaps_s]
    off_grid = sum(
        1 for value in frames
        if abs(value - round(value)) * period > OFF_GRID_TOLERANCE_S + FLOAT_SLACK_S)
    stats["off_grid_gaps"] = off_grid
    if off_grid:
        errors.append(
            f"{off_grid} of {len(frames)} stamp gaps are not a whole number of "
            f"{period * 1000:.1f} ms frames to within {OFF_GRID_TOLERANCE_S * 1000:.0f} ms")
    gap_frames = [max(1, round(value)) for value in frames]
    span_s = (cloud_stamps_ns[-1] - cloud_stamps_ns[0]) * 1e-9
    stats.update({
        "gaps": len(gap_frames),
        "mean_gap_frames": round(statistics.mean(gap_frames), 3),
        "median_gap_frames": statistics.median_low(gap_frames),
        "p95_gap_frames": percentile(gap_frames, 0.95),
        "longest_gap_frames": max(gap_frames),
        "rate_hz": round((len(cloud_stamps_ns) - 1) / span_s, 3),
        "delivered_fraction": round(len(gap_frames) / sum(gap_frames), 4),
    })

    latency = statistics.median(cloud_latencies_s)
    stats["median_latency_ms"] = round(latency * 1000.0, 2)
    stats["p95_latency_ms"] = round(percentile(cloud_latencies_s, 0.95) * 1000.0, 2)
    if depth_latencies_s:
        stats["median_depth_latency_ms"] = round(
            statistics.median(depth_latencies_s) * 1000.0, 2)

    if profile_name == "ideal":
        stats["gaps_over_one_frame"] = sum(1 for gap in gap_frames if gap != 1)
        # Every frame becomes a cloud, but a 600 kB Best Effort message can be
        # lost between processes: about 0.7% were on a quiet host. That is the
        # transport, not the profile, so a few lost clouds are allowed.
        if stats["delivered_fraction"] < IDEAL_DELIVERED_FRACTION:
            errors.append(
                f"ideal delivers every frame, but only {stats['delivered_fraction']:.1%} "
                f"of the camera's frames arrived as clouds, fewer than "
                f"{IDEAL_DELIVERED_FRACTION:.0%}")
        if stats["longest_gap_frames"] > IDEAL_LONGEST_GAP:
            errors.append(
                f"ideal delivers every frame, but a gap of {stats['longest_gap_frames']} "
                f"frames, longer than {IDEAL_LONGEST_GAP}, was seen")
        if depth_latencies_s and latency > statistics.median(depth_latencies_s) + IDEAL_LAG_S:
            errors.append(
                f"ideal adds no latency, but the cloud lags the depth image: median "
                f"{latency * 1000:.1f} ms against {stats['median_depth_latency_ms']} ms")
        return errors, stats

    table = normalized(profile["gap_frames"])
    count = len(gap_frames)
    if count < MINIMUM_GAPS:
        errors.append(
            f"only {count} gaps, fewer than the {MINIMUM_GAPS} that grade a distribution")
        return errors, stats
    for label, fraction in (("median", 0.5), ("p95", 0.95)):
        low, high = quantile_interval(table, fraction, count, DEVIATIONS)
        observed = stats["median_gap_frames"] if fraction == 0.5 else stats["p95_gap_frames"]
        stats[f"{label}_gap_interval"] = [low, high]
        if not low <= observed <= high:
            errors.append(
                f"{label} gap is {observed} frames; {DEVIATIONS:.0f} standard deviations "
                f"of {count} draws from the measured table allow {low} to {high}")
    for gap in (1, 2):
        expected = cumulative_within(table, gap)
        spread = DEVIATIONS * math.sqrt(expected * (1.0 - expected) / count)
        observed = sum(1 for value in gap_frames if value <= gap) / count
        if abs(observed - expected) > spread:
            errors.append(
                f"{observed:.3f} of gaps are at most {gap} frames; the table gives "
                f"{expected:.3f} +/- {spread:.3f} for {count} draws")
    longest = max(table)
    if stats["longest_gap_frames"] > longest + LONGEST_GAP_MARGIN:
        errors.append(
            f"longest gap is {stats['longest_gap_frames']} frames; the table's is {longest}")

    target = profile["latency_s"]
    if not target - LATENCY_BELOW_S <= latency <= target + LATENCY_ABOVE_S:
        errors.append(
            f"median latency is {latency * 1000:.1f} ms, expected "
            f"{target * 1000:.0f} ms (-{LATENCY_BELOW_S * 1000:.0f} "
            f"+{LATENCY_ABOVE_S * 1000:.0f})")
    stats["late_fraction"] = round(
        sum(1 for value in cloud_latencies_s if value > target + LATE_MARGIN_S)
        / len(cloud_latencies_s), 4)
    return errors, stats


class CloudTimingProbe(Node):
    """Record cloud and image stamps and their sim-time receipt, then grade them."""

    def __init__(self):
        super().__init__("cloud_timing_probe")
        self.declare_parameter("profile", "physical")
        self.declare_parameter("duration_s", 20.0)
        self.declare_parameter("timeout", 150.0)
        self.declare_parameter("report_path", "")
        self.profile_name = str(self.get_parameter("profile").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.report_path = str(self.get_parameter("report_path").value)
        self.profile = load_sensor_profile(default_path(), "point_cloud", self.profile_name)

        self.cloud_stamps = []
        self.cloud_latencies = []
        self.image_stamps = []
        self.depth_latencies = []
        self.depth_stamps = []
        self.layout = []
        qos = QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos)
        self.create_subscription(CameraInfo, FRAME_TOPIC, self.on_frame, qos)
        if self.profile_name == "ideal":
            # The ideal profile is graded against the depth image built from
            # the same frame, so it needs that image's latency.
            self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos)
        self.get_logger().info(
            f"Grading {self.profile_name} cloud timing over {self.duration_s:.0f} s of sim time")

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def on_cloud(self, message):
        stamp = self.stamp_ns(message)
        self.cloud_stamps.append(stamp)
        self.cloud_latencies.append((self.now_ns() - stamp) * 1e-9)
        if len(self.layout) < 3:
            self.layout.append(layout_errors(message))

    def on_frame(self, message):
        self.image_stamps.append(self.stamp_ns(message))

    def on_depth(self, message):
        self.depth_stamps.append(self.stamp_ns(message))
        self.depth_latencies.append((self.now_ns() - self.stamp_ns(message)) * 1e-9)

    def complete(self):
        return (
            bool(self.cloud_stamps) and bool(self.image_stamps)
            and (self.image_stamps[-1] - self.cloud_stamps[0]) * 1e-9 >= self.duration_s)

    def validate(self):
        errors = []
        if not self.cloud_stamps:
            return ["no point cloud arrived"], {}
        for problems in self.layout:
            errors.extend(f"layout: {problem}" for problem in problems[:3])
            if problems:
                break
        problems, stats = grade_timing(
            self.cloud_stamps, self.cloud_latencies, self.image_stamps, self.depth_stamps,
            self.depth_latencies, self.profile_name, self.profile)
        return errors + problems, stats


def main():
    rclpy.init()
    node = CloudTimingProbe()
    try:
        if node.duration_s <= 0.0:
            node.get_logger().info("Cloud timing grading skipped: duration_s is 0")
            return 0
        deadline = time.monotonic() + node.timeout
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
        errors, stats = node.validate()
        if node.report_path:
            with open(node.report_path, "w", encoding="utf-8") as report:
                json.dump({
                    "profile": node.profile_name,
                    "statistics": stats,
                    "errors": errors,
                    "stamps_ns": node.cloud_stamps,
                    "latencies_s": node.cloud_latencies,
                    "image_stamps_ns": node.image_stamps,
                    "depth_stamps_ns": node.depth_stamps,
                }, report)
        if errors:
            node.get_logger().error("Cloud timing FAILED: " + "; ".join(errors))
            node.get_logger().error(f"Statistics: {stats}")
            return 1
        node.get_logger().info(f"Cloud timing PASSED: {stats}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
