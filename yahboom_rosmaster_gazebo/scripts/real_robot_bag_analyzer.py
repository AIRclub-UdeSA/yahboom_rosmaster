#!/usr/bin/env python3
"""
Derive a topic/frame/rate/noise contract from recorded real-robot bags.

Reads one or more ROS 2 bags directly with rosbag2_py (never full playback),
computes per-topic rate, frame, and noise statistics, and can emit a
machine-readable contract YAML consumed by simulator parity work. Designed to
be re-run against future bags, including a smaller controlled calibration
capture, without code changes.
"""

import argparse
import math
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STRUCTURAL_TOPICS = [
    "/cmd_vel",
    "/odom",
    "/joint_states",
    "/scan",
    "/imu/data",
    "/camera_info",
    "/tf_static",
    "/robot_description",
]


def open_reader(bag_path, topics):
    """Open a filtered SequentialReader over only the requested topics."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def iter_messages(bag_path, topics):
    """Yield (topic, message, bag_timestamp_ns) for the requested topics."""
    reader, topic_types = open_reader(bag_path, topics)
    message_classes = {
        topic: get_message(type_str)
        for topic, type_str in topic_types.items()
        if topic in topics
    }
    while reader.has_next():
        topic, raw, bag_t = reader.read_next()
        message_class = message_classes.get(topic)
        if message_class is None:
            continue
        yield topic, deserialize_message(raw, message_class), bag_t


def header_seconds(message):
    """Return a header timestamp in seconds, or None if unavailable/zero."""
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return seconds if seconds > 0.0 else None


def rate_from_stamps(stamps):
    """Return the median sample rate in Hz from a strictly-increasing-ish list."""
    periods = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not periods:
        return None
    return 1.0 / statistics.median(periods)


def percentiles(values, points=(10, 50, 90)):
    """Return a dict of requested percentiles, or None if too few samples."""
    if len(values) < 2:
        return None
    ordered = sorted(values)
    result = {}
    for point in points:
        index = min(len(ordered) - 1, max(0, round((point / 100.0) * (len(ordered) - 1))))
        result[f"p{point}"] = ordered[index]
    return result


def quat_to_rpy(x, y, z, w):
    """Convert a quaternion to roll/pitch/yaw radians (ZYX convention)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class TopicAccumulator:
    """Collects generic rate/frame stats plus caller-supplied named metrics."""

    def __init__(self):
        self.count = 0
        self.stamps = []
        self.bag_stamps_s = []
        self.frame_ids = set()
        self.metrics = {}

    def note(self, message, bag_t):
        self.count += 1
        self.bag_stamps_s.append(bag_t * 1e-9)
        header_s = header_seconds(message)
        if header_s is not None:
            self.stamps.append(header_s)
        frame_id = getattr(getattr(message, "header", None), "frame_id", None)
        if frame_id:
            self.frame_ids.add(frame_id)

    def metric(self, name, value):
        self.metrics.setdefault(name, []).append(value)

    def rate_hz(self):
        return rate_from_stamps(self.stamps) or rate_from_stamps(self.bag_stamps_s)

    def summary(self):
        out = {
            "message_count": self.count,
            "frame_ids": sorted(self.frame_ids),
            "rate_hz": self.rate_hz(),
        }
        for name, values in self.metrics.items():
            if name.startswith("_"):
                continue
            numeric = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
            if not numeric:
                continue
            entry = {"median": statistics.median(numeric)}
            bounds = percentiles(numeric)
            if bounds:
                entry.update(bounds)
            out[name] = entry
        return out


def analyze_cmd_vel(accum, msg, bag_t):
    accum.count += 1
    accum.bag_stamps_s.append(bag_t * 1e-9)
    accum.metric("linear_x", msg.linear.x)
    accum.metric("linear_y", msg.linear.y)
    accum.metric("angular_z", msg.angular.z)


def analyze_odom(accum, msg, bag_t):
    accum.note(msg, bag_t)
    accum.metric("twist_linear_x", msg.twist.twist.linear.x)
    accum.metric("twist_linear_y", msg.twist.twist.linear.y)
    accum.metric("twist_angular_z", msg.twist.twist.angular.z)
    accum.metric("pose_cov_xx", msg.pose.covariance[0])
    accum.metric("pose_cov_yy", msg.pose.covariance[7])
    accum.metric("pose_cov_yawyaw", msg.pose.covariance[35])
    accum.metrics.setdefault("_frame_ids_extra", set()).add(
        (msg.header.frame_id, msg.child_frame_id))
    accum.metrics.setdefault("_xy", []).append((
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
    ))


def path_length(xy_samples):
    total = 0.0
    for (x0, y0), (x1, y1) in zip(xy_samples, xy_samples[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def analyze_joint_states(accum, msg, bag_t):
    accum.note(msg, bag_t)
    accum.metrics.setdefault("_names", set()).update(msg.name)
    all_zero = all(p == 0.0 for p in msg.position)
    accum.metrics.setdefault("_all_positions_zero", []).append(all_zero)
    accum.metrics.setdefault("_has_velocity", []).append(bool(msg.velocity))


def analyze_scan(accum, msg, bag_t):
    accum.note(msg, bag_t)
    accum.metric("angle_min", msg.angle_min)
    accum.metric("angle_max", msg.angle_max)
    accum.metric("angle_increment", msg.angle_increment)
    accum.metric("range_min", msg.range_min)
    accum.metric("range_max", msg.range_max)
    accum.metric("scan_time", msg.scan_time)
    accum.metric("time_increment", msg.time_increment)
    accum.metric("ray_count", len(msg.ranges))
    non_finite = sum(1 for r in msg.ranges if not math.isfinite(r))
    accum.metric("non_finite_returns", non_finite)


def analyze_imu(accum, msg, bag_t):
    accum.note(msg, bag_t)
    accum.metric("accel_x", msg.linear_acceleration.x)
    accum.metric("accel_y", msg.linear_acceleration.y)
    accum.metric("accel_z", msg.linear_acceleration.z)
    accum.metric("gyro_x", msg.angular_velocity.x)
    accum.metric("gyro_y", msg.angular_velocity.y)
    accum.metric("gyro_z", msg.angular_velocity.z)
    accum.metrics.setdefault("_orientation_cov_diag", []).append(
        tuple(msg.orientation_covariance[i] for i in (0, 4, 8)))
    accum.metrics.setdefault("_angvel_cov_diag", []).append(
        tuple(msg.angular_velocity_covariance[i] for i in (0, 4, 8)))
    accum.metrics.setdefault("_accel_cov_diag", []).append(
        tuple(msg.linear_acceleration_covariance[i] for i in (0, 4, 8)))


def analyze_camera_info(accum, msg, bag_t):
    accum.note(msg, bag_t)
    accum.metrics.setdefault("_dims", set()).add((msg.width, msg.height))
    accum.metrics.setdefault("_k_all_zero", []).append(all(v == 0.0 for v in msg.k))
    accum.metrics.setdefault("_distortion_model", set()).add(msg.distortion_model)


def collect_tf_static(accum, msg, bag_t):
    accum.count += 1
    for transform in msg.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rpy = quat_to_rpy(rotation.x, rotation.y, rotation.z, rotation.w)
        accum.metrics.setdefault("_edges", {})[
            (transform.header.frame_id, transform.child_frame_id)
        ] = {
            "xyz": [translation.x, translation.y, translation.z],
            "rpy": list(rpy),
        }


def parse_urdf_geometry(urdf_text):
    """Extract joint origins from a flattened (non-xacro) recorded URDF."""
    root = ET.fromstring(urdf_text)
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
        joints[name] = {
            "type": joint.get("type"),
            "parent": parent.get("link") if parent is not None else None,
            "child": child.get("link") if child is not None else None,
            "xyz": xyz,
            "rpy": rpy,
        }
    return joints


def analyze_bag(bag_path, image_samples=3):
    """Return a per-bag statistics dict."""
    accumulators = {topic: TopicAccumulator() for topic in STRUCTURAL_TOPICS}
    handlers = {
        "/cmd_vel": analyze_cmd_vel,
        "/odom": analyze_odom,
        "/joint_states": analyze_joint_states,
        "/scan": analyze_scan,
        "/imu/data": analyze_imu,
        "/camera_info": analyze_camera_info,
        "/tf_static": collect_tf_static,
    }
    urdf_text = None
    for topic, msg, bag_t in iter_messages(bag_path, STRUCTURAL_TOPICS):
        if topic == "/robot_description":
            if urdf_text is None:
                urdf_text = msg.data
            continue
        handlers[topic](accumulators[topic], msg, bag_t)

    image_info = analyze_image_samples(bag_path, image_samples)

    result = {topic.strip("/").replace("/", "_"): accum.summary()
              for topic, accum in accumulators.items() if topic != "/robot_description"}

    odom_accum = accumulators["/odom"]
    if odom_accum.count:
        result["odom"]["reported_frames"] = sorted(
            f"{a} -> {b}" for a, b in odom_accum.metrics.get("_frame_ids_extra", set()))
        result["odom"]["path_length_m"] = path_length(odom_accum.metrics.get("_xy", []))

    joint_accum = accumulators["/joint_states"]
    if joint_accum.count:
        result["joint_states"]["names"] = sorted(joint_accum.metrics.get("_names", set()))
        result["joint_states"]["all_positions_always_zero"] = all(
            joint_accum.metrics.get("_all_positions_zero", [False]))
        result["joint_states"]["any_velocity_reported"] = any(
            joint_accum.metrics.get("_has_velocity", [False]))

    camera_info_accum = accumulators["/camera_info"]
    if camera_info_accum.count:
        result["camera_info"]["dimensions"] = sorted(
            camera_info_accum.metrics.get("_dims", set()))
        result["camera_info"]["k_always_zero"] = all(
            camera_info_accum.metrics.get("_k_all_zero", [False]))
        result["camera_info"]["distortion_models"] = sorted(
            m for m in camera_info_accum.metrics.get("_distortion_model", set()))

    imu_accum = accumulators["/imu/data"]
    if imu_accum.count:
        orientation_cov = imu_accum.metrics.get("_orientation_cov_diag", [])
        angvel_cov = imu_accum.metrics.get("_angvel_cov_diag", [])
        accel_cov = imu_accum.metrics.get("_accel_cov_diag", [])
        result["imu_data"]["orientation_covariance_diag"] = (
            list(orientation_cov[0]) if orientation_cov else None
        )
        result["imu_data"]["angular_velocity_covariance_diag"] = (
            list(angvel_cov[0]) if angvel_cov else None
        )
        result["imu_data"]["linear_acceleration_covariance_diag"] = (
            list(accel_cov[0]) if accel_cov else None
        )
        gravity_mag = [
            math.hypot(math.hypot(ax, ay), az)
            for ax, ay, az in zip(
                imu_accum.metrics.get("accel_x", []),
                imu_accum.metrics.get("accel_y", []),
                imu_accum.metrics.get("accel_z", []))
        ]
        if gravity_mag:
            result["imu_data"]["gravity_magnitude_median"] = statistics.median(gravity_mag)

    tf_static_edges = accumulators["/tf_static"].metrics.get("_edges", {})
    result["tf_static"] = {
        f"{parent} -> {child}": edge
        for (parent, child), edge in sorted(tf_static_edges.items())
    }

    result["image_raw"] = image_info
    result["urdf_geometry"] = parse_urdf_geometry(urdf_text) if urdf_text else None
    return result


def analyze_image_samples(bag_path, count):
    """Sample the first few /image_raw messages without scanning the topic."""
    if count <= 0:
        return {"sampled": 0}
    samples = []
    for _, msg, _ in iter_messages(bag_path, ["/image_raw"]):
        samples.append(msg)
        if len(samples) >= count:
            break
    if not samples:
        return {"sampled": 0}
    return {
        "sampled": len(samples),
        "encodings": sorted({s.encoding for s in samples}),
        "dimensions": sorted({(s.width, s.height) for s in samples}),
        "frame_ids": sorted({s.header.frame_id for s in samples}),
    }


def bag_duration_and_count(bag_path):
    """Read duration/message_count straight from metadata.yaml (cheap)."""
    metadata_path = Path(bag_path) / "metadata.yaml"
    with open(metadata_path) as f:
        info = yaml.safe_load(f)["rosbag2_bagfile_information"]
    return (
        info["duration"]["nanoseconds"] * 1e-9,
        info["message_count"],
    )


def build_report(bag_paths, image_samples):
    report = {"bags": {}}
    for bag_path in bag_paths:
        bag_id = Path(bag_path).name
        duration_s, message_count = bag_duration_and_count(bag_path)
        stats = analyze_bag(bag_path, image_samples=image_samples)
        report["bags"][bag_id] = {
            "path": str(bag_path),
            "duration_s": duration_s,
            "message_count": message_count,
            "topics": stats,
        }
    return report


def to_native(value):
    """Recursively convert numpy scalars/arrays and tuples for plain YAML output."""
    if isinstance(value, dict):
        return {to_native(k): to_native(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_native(v) for v in value]
    if hasattr(value, "item") and not isinstance(value, (str, bytes)):
        return value.item()
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", help="Paths to extracted rosbag2 directories")
    parser.add_argument("--output", type=Path, help="Write the full report as YAML to this path")
    parser.add_argument(
        "--image-samples", type=int, default=3,
        help="Number of leading /image_raw messages to sample for encoding/size (default 3)")
    args = parser.parse_args()

    report = to_native(build_report(args.bags, args.image_samples))

    if args.output:
        with open(args.output, "w") as f:
            yaml.safe_dump(report, f, sort_keys=False, default_flow_style=False)
        print(f"Wrote report for {len(args.bags)} bag(s) to {args.output}")
    else:
        print(yaml.safe_dump(report, sort_keys=False, default_flow_style=False))


if __name__ == "__main__":
    main()
