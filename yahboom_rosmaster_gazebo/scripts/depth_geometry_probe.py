#!/usr/bin/env python3
"""Validate RGB-D response against a known red box in the empty world."""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from real_robot_contract import RealRobotContract


COLOR_TOPIC = "/cam_1/color/image_raw"
DEPTH_TOPIC = "/cam_1/depth/image_raw"
COLOR_INFO_TOPIC = "/cam_1/color/camera_info"
DEPTH_INFO_TOPIC = "/cam_1/depth/camera_info"
POINTS_TOPIC = "/cam_1/depth/color/points"
DEPTH_FRAME = "cam_1_depth_frame"
DEPTH_OPTICAL_FRAME = "cam_1_depth_optical_frame"
COLOR_FRAME = "cam_1_color_frame"
COLOR_OPTICAL_FRAME = "cam_1_color_optical_frame"
# REP 103 optical rotation of each *_optical_frame from its parent frame.
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
# Resolved camera TF must reproduce the ledger's poses to 1 um and 1 urad.
CAMERA_TF_TOLERANCE = 1e-6
# The first color frames, before the target spawns, are the red baseline. At
# 30 Hz the probe keeps them plus a rolling window of the latest frames.
BASELINE_OBSERVATIONS = 3
MAX_OBSERVATIONS = 120
OFF_AXIS_COLUMN_OFFSET = 40
OFF_AXIS_ROW_OFFSET = 20
# A cloud point must land within this distance of where the depth image and
# the ledger's color-to-depth offset put it. The offset itself is 25.1 mm.
CLOUD_POINT_TOLERANCE = 0.005
# The red target's centroid must land within this many pixels of its
# projection from the image frame. Seen from the depth aperture instead, it
# would sit about 9 px away at 0.745 m.
TARGET_CENTROID_TOLERANCE = 2.0


def pose_matrix(xyz, rpy):
    """Return the homogeneous transform of a URDF xyz/rpy origin."""
    roll, pitch, yaw = rpy
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = (
        np.array(((cos_y, -sin_y, 0.0), (sin_y, cos_y, 0.0), (0.0, 0.0, 1.0)))
        @ np.array(((cos_p, 0.0, sin_p), (0.0, 1.0, 0.0), (-sin_p, 0.0, cos_p)))
        @ np.array(((1.0, 0.0, 0.0), (0.0, cos_r, -sin_r), (0.0, sin_r, cos_r)))
    )
    matrix[:3, 3] = xyz
    return matrix


def transform_matrix(transform):
    """Return the homogeneous matrix of a geometry_msgs Transform."""
    x, y, z, w = (
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix = np.eye(4)
    matrix[:3, :3] = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


class DepthGeometryProbe(Node):
    """Collect synchronized camera samples and validate known scene geometry."""

    def __init__(self):
        super().__init__("depth_geometry_probe")
        self.declare_parameter("timeout", 45.0)
        self.declare_parameter("samples", 10)
        # depth_geometry.launch.py derives these from the ledger's camera
        # mounts: the depth of the target's front face along the image
        # frame's optical axis, and the face's centre on base_link.
        self.declare_parameter("expected_depth", 0.745)
        self.declare_parameter("target_face_center", [0.8, 0.0, 0.0374])
        self.declare_parameter("depth_tolerance", 0.03)
        self.declare_parameter("target_red_pixels", 500)
        self.declare_parameter("minimum_valid_fraction", 0.02)

        self.timeout = float(self.get_parameter("timeout").value)
        self.samples = max(10, int(self.get_parameter("samples").value))
        self.expected_depth = float(
            self.get_parameter("expected_depth").value)
        self.target_face_center = np.array(
            self.get_parameter("target_face_center").value, dtype=float)
        self.depth_tolerance = float(
            self.get_parameter("depth_tolerance").value)
        self.target_red_pixels = int(
            self.get_parameter("target_red_pixels").value)
        self.minimum_valid_fraction = float(
            self.get_parameter("minimum_valid_fraction").value)

        # Registered RGB-D: all four image and camera_info topics share one
        # optical frame (real_robot_contract_test.py checks the ledger for it),
        # so the depth image's frame applies to every one of them.
        contract = RealRobotContract.load()
        self.expected_width = contract.nominal("camera.width")
        self.expected_height = contract.nominal("camera.height")
        self.expected_image_frame = contract.nominal(
            f"topics.{DEPTH_TOPIC}.frame_id")
        self.expected_cloud_frame = contract.nominal(
            f"topics.{POINTS_TOPIC}.frame_id")
        self.depth_near = contract.nominal("depth.min_range_m")
        self.depth_far = contract.nominal("depth.max_range_m")
        # Camera frame poses on base_link, from the ledger's mounts. The color
        # frames sit at the physical unit's calibrated offset from depth.
        optical = pose_matrix((0.0, 0.0, 0.0), OPTICAL_RPY)
        self.camera_poses = {}
        for sensor in ("depth", "color"):
            frame = f"cam_1_{sensor}_frame"
            pose = pose_matrix(**contract.nominal(f"frames.mounts.{frame}"))
            self.camera_poses[frame] = pose
            self.camera_poses[f"cam_1_{sensor}_optical_frame"] = pose @ optical

        self.observations = {
            COLOR_TOPIC: [],
            DEPTH_TOPIC: [],
            COLOR_INFO_TOPIC: [],
            DEPTH_INFO_TOPIC: [],
            POINTS_TOPIC: [],
        }
        self.capture_errors = []
        self.parallax_report = "target parallax not checked"
        self._subscription_handles = []

        sensor_qos = QoSProfile(
            depth=40,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        subscriptions = (
            (Image, COLOR_TOPIC, self.capture_color),
            (Image, DEPTH_TOPIC, self.capture_depth),
            (CameraInfo, COLOR_INFO_TOPIC, self.capture_color_info),
            (CameraInfo, DEPTH_INFO_TOPIC, self.capture_depth_info),
            (PointCloud2, POINTS_TOPIC, self.capture_points),
        )
        for message_type, topic, callback in subscriptions:
            self._subscription_handles.append(
                self.create_subscription(
                    message_type, topic, callback, sensor_qos))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0), node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.get_logger().info(
            f"Waiting up to {self.timeout:.1f}s for {self.samples} coherent "
            f"RGB-D target samples at {self.expected_depth:.3f}m")

    @staticmethod
    def stamp_ns(message):
        """Return a message header stamp as integer nanoseconds."""
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def record_error(self, topic, exception):
        """Record callback parsing errors without flooding the launch log."""
        error = f"{topic}: {type(exception).__name__}: {exception}"
        if error not in self.capture_errors:
            self.capture_errors.append(error)
            self.get_logger().error(error)

    def append(self, topic, observation):
        """Retain the baseline frames and a rolling window of recent ones."""
        observations = self.observations[topic]
        if len(observations) >= MAX_OBSERVATIONS:
            del observations[BASELINE_OBSERVATIONS]
        observations.append(observation)

    @staticmethod
    def image_array(message, channels, dtype):
        """Create a strided NumPy view that honors ROS Image row padding."""
        item_size = np.dtype(dtype).itemsize
        required_step = int(message.width) * channels * item_size
        if message.step < required_step:
            raise ValueError(
                f"step {message.step} is smaller than {required_step}")
        if len(message.data) < message.step * message.height:
            raise ValueError("data is shorter than step*height")
        if channels == 1:
            strides = (message.step, item_size)
            shape = (message.height, message.width)
        else:
            strides = (message.step, channels * item_size, item_size)
            shape = (message.height, message.width, channels)
        return np.ndarray(
            shape=shape, dtype=dtype, buffer=message.data, strides=strides)

    def capture_color(self, message):
        try:
            red_pixels = None
            red_centroid = None
            if message.encoding == "rgb8":
                image = self.image_array(message, 3, np.uint8).astype(
                    np.int16, copy=False)
                red_mask = (
                    (image[:, :, 0] > 60)
                    & (image[:, :, 0] > image[:, :, 1] + 40)
                    & (image[:, :, 0] > image[:, :, 2] + 40)
                )
                red_pixels = int(np.count_nonzero(red_mask))
                if red_pixels:
                    rows, columns = np.nonzero(red_mask)
                    red_centroid = (float(columns.mean()), float(rows.mean()))
            self.append(COLOR_TOPIC, {
                "stamp": self.stamp_ns(message),
                "frame": message.header.frame_id,
                "encoding": message.encoding,
                "width": int(message.width),
                "height": int(message.height),
                "data_size": len(message.data),
                "expected_data_size": int(message.step * message.height),
                "red_pixels": red_pixels,
                "red_centroid": red_centroid,
            })
        except Exception as exception:  # ROS callbacks must remain alive.
            self.record_error(COLOR_TOPIC, exception)

    def capture_depth(self, message):
        try:
            center_depth = None
            off_axis_depth = None
            valid_fraction = 0.0
            finite_nonpositive = 0
            valid_min = None
            valid_max = None
            if message.encoding == "32FC1":
                endian = ">f4" if message.is_bigendian else "<f4"
                depth = self.image_array(message, 1, np.dtype(endian))
                finite = np.isfinite(depth)
                valid = finite & (depth > 0.0)
                center_depth = float(
                    depth[message.height // 2, message.width // 2])
                off_axis_depth = float(depth[
                    message.height // 2 + OFF_AXIS_ROW_OFFSET,
                    message.width // 2 + OFF_AXIS_COLUMN_OFFSET,
                ])
                valid_fraction = float(np.count_nonzero(valid) / depth.size)
                finite_nonpositive = int(np.count_nonzero(finite & ~valid))
                if np.any(valid):
                    valid_min = float(np.min(depth[valid]))
                    valid_max = float(np.max(depth[valid]))
            self.append(DEPTH_TOPIC, {
                "stamp": self.stamp_ns(message),
                "frame": message.header.frame_id,
                "encoding": message.encoding,
                "width": int(message.width),
                "height": int(message.height),
                "data_size": len(message.data),
                "expected_data_size": int(message.step * message.height),
                "center_depth": center_depth,
                "off_axis_depth": off_axis_depth,
                "valid_fraction": valid_fraction,
                "finite_nonpositive": finite_nonpositive,
                "valid_min": valid_min,
                "valid_max": valid_max,
            })
        except Exception as exception:  # ROS callbacks must remain alive.
            self.record_error(DEPTH_TOPIC, exception)

    def capture_info(self, topic, message):
        try:
            self.append(topic, {
                "stamp": self.stamp_ns(message),
                "frame": message.header.frame_id,
                "width": int(message.width),
                "height": int(message.height),
                "k": tuple(float(value) for value in message.k),
                "p": tuple(float(value) for value in message.p),
            })
        except Exception as exception:  # ROS callbacks must remain alive.
            self.record_error(topic, exception)

    def capture_color_info(self, message):
        self.capture_info(COLOR_INFO_TOPIC, message)

    def capture_depth_info(self, message):
        self.capture_info(DEPTH_INFO_TOPIC, message)

    def capture_points(self, message):
        try:
            fields = {field.name: field for field in message.fields}
            field_metadata = {
                name: (int(field.offset), int(field.datatype), int(field.count))
                for name, field in fields.items()
            }
            center_xyz = None
            off_axis_xyz = None
            center_rgb = None
            valid_fraction = 0.0
            if all(
                    name in fields
                    and fields[name].datatype == PointField.FLOAT32
                    and fields[name].count == 1
                    for name in ("x", "y", "z")):
                endian = ">f4" if message.is_bigendian else "<f4"
                coordinates = []
                for name in ("x", "y", "z"):
                    field = fields[name]
                    if field.offset + 4 > message.point_step:
                        raise ValueError(
                            f"{name} field exceeds point_step {message.point_step}")
                    coordinates.append(np.ndarray(
                        shape=(message.height, message.width),
                        dtype=np.dtype(endian),
                        buffer=message.data,
                        offset=field.offset,
                        strides=(message.row_step, message.point_step),
                    ))
                valid = np.logical_and.reduce(
                    [np.isfinite(array) for array in coordinates])
                valid_fraction = float(
                    np.count_nonzero(valid) / valid.size)
                row = message.height // 2
                column = message.width // 2
                center_xyz = tuple(
                    float(array[row, column]) for array in coordinates)
                off_axis_xyz = tuple(float(array[
                    row + OFF_AXIS_ROW_OFFSET,
                    column + OFF_AXIS_COLUMN_OFFSET,
                ]) for array in coordinates)
            if (
                    "rgb" in fields
                    and fields["rgb"].datatype == PointField.FLOAT32
                    and fields["rgb"].count == 1
                    and fields["rgb"].offset + 4 <= message.point_step):
                integer_endian = ">u4" if message.is_bigendian else "<u4"
                packed_rgb = np.ndarray(
                    shape=(message.height, message.width),
                    dtype=np.dtype(integer_endian),
                    buffer=message.data,
                    offset=fields["rgb"].offset,
                    strides=(message.row_step, message.point_step),
                )
                packed = int(packed_rgb[
                    message.height // 2, message.width // 2])
                center_rgb = (
                    (packed >> 16) & 0xFF,
                    (packed >> 8) & 0xFF,
                    packed & 0xFF,
                )
            self.append(POINTS_TOPIC, {
                "stamp": self.stamp_ns(message),
                "frame": message.header.frame_id,
                "width": int(message.width),
                "height": int(message.height),
                "point_step": int(message.point_step),
                "row_step": int(message.row_step),
                "data_size": len(message.data),
                "fields": field_metadata,
                "center_xyz": center_xyz,
                "off_axis_xyz": off_axis_xyz,
                "center_rgb": center_rgb,
                "valid_fraction": valid_fraction,
            })
        except Exception as exception:  # ROS callbacks must remain alive.
            self.record_error(POINTS_TOPIC, exception)

    def depth_matches_target(self, observation):
        depth = observation["center_depth"]
        return (
            depth is not None
            and math.isfinite(depth)
            and math.isclose(
                depth, self.expected_depth, abs_tol=self.depth_tolerance)
        )

    def cloud_contains_target(self, observation):
        """Detect arrival independent of axis convention; validation is strict."""
        xyz = observation["center_xyz"]
        return (
            xyz is not None
            and any(
                math.isfinite(value)
                and math.isclose(
                    value, self.expected_depth, abs_tol=self.depth_tolerance)
                for value in xyz
            )
        )

    def coherent_target_stamps(self):
        """Return exact sensor-cycle stamps shared by all five RGB-D streams."""
        stamp_sets = [
            {
                item["stamp"] for item in self.observations[COLOR_TOPIC]
                if item["red_pixels"] is not None
                and item["red_pixels"] >= self.target_red_pixels
            },
            {
                item["stamp"] for item in self.observations[DEPTH_TOPIC]
                if self.depth_matches_target(item)
            },
            {
                item["stamp"] for item in self.observations[POINTS_TOPIC]
                if self.cloud_contains_target(item)
            },
            {
                item["stamp"]
                for item in self.observations[COLOR_INFO_TOPIC]
            },
            {
                item["stamp"]
                for item in self.observations[DEPTH_INFO_TOPIC]
            },
        ]
        return sorted(set.intersection(*stamp_sets))

    def complete(self):
        if any(
                len(items) < self.samples
                for items in self.observations.values()):
            return False
        if (
                len(self.observations[COLOR_TOPIC])
                < self.samples + BASELINE_OBSERVATIONS):
            return False
        return len(self.coherent_target_stamps()) >= self.samples

    @staticmethod
    def validate_stamps(topic, observations, errors):
        stamps = [item["stamp"] for item in observations]
        if any(stamp <= 0 for stamp in stamps):
            errors.append(f"{topic}: zero or negative header stamp")
        if any(
                current <= previous
                for previous, current in zip(stamps, stamps[1:])):
            errors.append(f"{topic}: stamps are not strictly increasing")

    def validate_common_image_contract(self, topic, observations, encoding,
                                       errors):
        for item in observations:
            if item["encoding"] != encoding:
                errors.append(
                    f"{topic}: expected {encoding}, got {item['encoding']}")
                break
            if (item["width"], item["height"]) != (
                    self.expected_width, self.expected_height):
                errors.append(
                    f"{topic}: expected {self.expected_width}x"
                    f"{self.expected_height}, "
                    f"got {item['width']}x{item['height']}")
                break
            if item["data_size"] != item["expected_data_size"]:
                errors.append(f"{topic}: data length does not equal step*height")
                break
            if item["frame"] != self.expected_image_frame:
                errors.append(
                    f"{topic}: expected frame {self.expected_image_frame}, "
                    f"got {item['frame']}")
                break

    def validate_registered_frame_contract(self, coherent_stamps, errors):
        """Validate stream headers and the calibrated color/depth offset."""
        if not coherent_stamps:
            return

        samples_by_topic = {
            topic: {item["stamp"]: item for item in self.observations[topic]}
            for topic in (
                COLOR_TOPIC,
                DEPTH_TOPIC,
                COLOR_INFO_TOPIC,
                DEPTH_INFO_TOPIC,
            )
        }
        for stamp in coherent_stamps[-self.samples:]:
            color = samples_by_topic[COLOR_TOPIC][stamp]
            depth = samples_by_topic[DEPTH_TOPIC][stamp]
            color_info = samples_by_topic[COLOR_INFO_TOPIC][stamp]
            depth_info = samples_by_topic[DEPTH_INFO_TOPIC][stamp]
            if color["frame"] != depth["frame"]:
                errors.append(
                    "registered camera frames: color and depth images at "
                    f"{stamp} use {color['frame']} and {depth['frame']}")
                break
            if color_info["frame"] != color["frame"]:
                errors.append(
                    "registered camera frames: color CameraInfo does not "
                    f"match its image header at {stamp}")
                break
            if depth_info["frame"] != depth["frame"]:
                errors.append(
                    "registered camera frames: depth CameraInfo does not "
                    f"match its image header at {stamp}")
                break

        stamp = Time(nanoseconds=coherent_stamps[-1])
        for depth_frame, color_frame in (
                (DEPTH_FRAME, COLOR_FRAME),
                (DEPTH_OPTICAL_FRAME, COLOR_OPTICAL_FRAME)):
            try:
                transform = self.tf_buffer.lookup_transform(
                    depth_frame,
                    color_frame,
                    stamp,
                    timeout=Duration(seconds=0.2),
                ).transform
            except TransformException as exception:
                errors.append(
                    "registered camera TF: cannot resolve "
                    f"{depth_frame} <- {color_frame} at the sensor stamp: "
                    f"{exception}")
                continue

            actual = transform_matrix(transform)
            expected = (
                np.linalg.inv(self.camera_poses[depth_frame])
                @ self.camera_poses[color_frame])
            translation_error = np.linalg.norm(actual[:3, 3] - expected[:3, 3])
            cosine = (np.trace(actual[:3, :3].T @ expected[:3, :3]) - 1.0) / 2.0
            rotation_error = math.acos(max(-1.0, min(1.0, cosine)))
            if max(translation_error, rotation_error) > CAMERA_TF_TOLERANCE:
                errors.append(
                    "registered camera TF: "
                    f"{depth_frame} <- {color_frame} is "
                    f"{translation_error * 1000:.4f} mm and "
                    f"{rotation_error:.2e} rad from the ledger's calibrated "
                    f"offset; got translation {actual[:3, 3].tolist()}, "
                    f"expected {expected[:3, 3].tolist()}")

    def project_target(self, frame, k):
        """Return the target face centre's pixel as seen from an optical frame."""
        point = np.linalg.inv(self.camera_poses[frame]) @ np.append(
            self.target_face_center, 1.0)
        return (
            k[0] * point[0] / point[2] + k[2],
            k[4] * point[1] / point[2] + k[5],
        )

    def validate_target_parallax(self, coherent_stamps, errors):
        """Require the red target where the image frame's aperture sees it."""
        colors = {
            item["stamp"]: item for item in self.observations[COLOR_TOPIC]}
        infos = {
            item["stamp"]: item for item in self.observations[COLOR_INFO_TOPIC]}
        for stamp in coherent_stamps[-self.samples:]:
            centroid = colors[stamp]["red_centroid"]
            k = infos[stamp]["k"]
            expected = self.project_target(self.expected_image_frame, k)
            self.parallax_report = (
                f"red target centroid ({centroid[0]:.2f}, {centroid[1]:.2f}) "
                f"px, predicted ({expected[0]:.2f}, {expected[1]:.2f}) from "
                f"{self.expected_image_frame}")
            if max(
                    abs(actual - predicted)
                    for actual, predicted in zip(centroid, expected)
            ) > TARGET_CENTROID_TOLERANCE:
                other = self.project_target(DEPTH_OPTICAL_FRAME, k)
                errors.append(
                    f"target parallax: {self.parallax_report} is off by more "
                    f"than {TARGET_CENTROID_TOLERANCE} px; from "
                    f"{DEPTH_OPTICAL_FRAME} it would be at "
                    f"({other[0]:.2f}, {other[1]:.2f})")
                return

    def expected_cloud_point(self, column, row, depth, k):
        """Return where a depth pixel should land in the cloud's frame."""
        optical = np.array((
            (column - k[2]) * depth / k[0],
            (row - k[5]) * depth / k[4],
            depth,
            1.0,
        ))
        cloud_from_image = (
            np.linalg.inv(self.camera_poses[self.expected_cloud_frame])
            @ self.camera_poses[self.expected_image_frame])
        return (cloud_from_image @ optical)[:3]

    def validate_cloud_geometry(self, coherent_stamps, errors):
        """Require each sampled pixel's cloud point where its depth puts it."""
        depths = {
            item["stamp"]: item for item in self.observations[DEPTH_TOPIC]}
        clouds = {
            item["stamp"]: item for item in self.observations[POINTS_TOPIC]}
        infos = {
            item["stamp"]: item for item in self.observations[DEPTH_INFO_TOPIC]}
        for label, row_offset, column_offset, depth_key, cloud_key in (
                ("centre", 0, 0, "center_depth", "center_xyz"),
                ("lower-right", OFF_AXIS_ROW_OFFSET, OFF_AXIS_COLUMN_OFFSET,
                 "off_axis_depth", "off_axis_xyz")):
            matches = 0
            first_mismatch = None
            for stamp in coherent_stamps[-self.samples:]:
                depth_sample = depths[stamp]
                depth = depth_sample[depth_key]
                xyz = clouds[stamp][cloud_key]
                expected = self.expected_cloud_point(
                    depth_sample["width"] // 2 + column_offset,
                    depth_sample["height"] // 2 + row_offset,
                    depth,
                    infos[stamp]["k"])
                if (
                        xyz is not None
                        and math.isfinite(depth)
                        and np.linalg.norm(np.array(xyz) - expected)
                        <= CLOUD_POINT_TOLERANCE):
                    matches += 1
                elif first_mismatch is None:
                    first_mismatch = (xyz, np.round(expected, 4).tolist())
            if matches < self.samples:
                errors.append(
                    f"point cloud geometry: {matches}/{self.samples} {label} "
                    f"points lie within {CLOUD_POINT_TOLERANCE * 1000:.0f} mm "
                    "of the depth image's point expressed in "
                    f"{self.expected_cloud_frame} (+X forward, +Y left, +Z "
                    f"up); first cloud/expected XYZ was {first_mismatch}")

    def validate(self):
        errors = list(self.capture_errors)
        for topic, observations in self.observations.items():
            if len(observations) < self.samples:
                errors.append(
                    f"{topic}: received {len(observations)}/{self.samples} "
                    "required messages")
            self.validate_stamps(topic, observations, errors)
        if any(len(items) < self.samples for items in self.observations.values()):
            return errors

        colors = self.observations[COLOR_TOPIC]
        depths = self.observations[DEPTH_TOPIC]
        color_info = self.observations[COLOR_INFO_TOPIC]
        depth_info = self.observations[DEPTH_INFO_TOPIC]
        clouds = self.observations[POINTS_TOPIC]
        self.validate_common_image_contract(
            COLOR_TOPIC, colors, "rgb8", errors)
        self.validate_common_image_contract(
            DEPTH_TOPIC, depths, "32FC1", errors)

        baseline = colors[:BASELINE_OBSERVATIONS]
        baseline_red = [item["red_pixels"] for item in baseline]
        target_colors = [
            item for item in colors
            if item["red_pixels"] is not None
            and item["red_pixels"] >= self.target_red_pixels
        ]
        if any(value is None for value in baseline_red):
            errors.append("color response: baseline rgb8 pixels were not decoded")
        elif len(target_colors) < self.samples:
            errors.append(
                f"color response: only {len(target_colors)}/{self.samples} "
                f"frames contain at least {self.target_red_pixels} red pixels")
        else:
            baseline_max = max(baseline_red)
            target_min = min(
                item["red_pixels"] for item in target_colors[-self.samples:])
            if target_min < baseline_max + self.target_red_pixels:
                errors.append(
                    "color response: red target did not change enough pixels "
                    f"(baseline max {baseline_max}, target min {target_min})")

        target_depths = [
            item for item in depths if self.depth_matches_target(item)]
        if len(target_depths) < self.samples:
            center_values = [item["center_depth"] for item in depths[-3:]]
            errors.append(
                f"depth geometry: only {len(target_depths)}/{self.samples} "
                f"center samples match {self.expected_depth:.3f}+/-"
                f"{self.depth_tolerance:.3f}m; latest {center_values}")
        for item in target_depths[-self.samples:]:
            if item["valid_fraction"] < self.minimum_valid_fraction:
                errors.append(
                    "depth coverage: only "
                    f"{item['valid_fraction']:.1%} pixels are finite and positive")
                break
            if item["finite_nonpositive"]:
                errors.append(
                    "depth validity: finite zero/negative depth values observed")
                break
            if (
                    item["valid_min"] is None
                    or item["valid_min"] < self.depth_near - 0.01
                    or item["valid_max"] > self.depth_far + 0.01):
                errors.append(
                    "depth validity: finite values fall outside configured "
                    f"clip range ({item['valid_min']}, {item['valid_max']})")
                break

        for topic, info_samples in (
                (COLOR_INFO_TOPIC, color_info),
                (DEPTH_INFO_TOPIC, depth_info)):
            for item in info_samples:
                if (item["width"], item["height"]) != (
                        self.expected_width, self.expected_height):
                    errors.append(f"{topic}: dimensions do not match images")
                    break
                if item["frame"] != self.expected_image_frame:
                    errors.append(
                        f"{topic}: expected frame {self.expected_image_frame}, "
                        f"got {item['frame']}")
                    break
                if (
                        len(item["k"]) != 9
                        or len(item["p"]) != 12
                        or not all(math.isfinite(value) for value in item["k"])
                        or not all(math.isfinite(value) for value in item["p"])):
                    errors.append(f"{topic}: invalid camera matrices")
                    break

        target_clouds = [
            item for item in clouds if self.cloud_contains_target(item)]
        if len(target_clouds) < self.samples:
            errors.append(
                f"point cloud: only {len(target_clouds)}/{self.samples} center "
                "points contain the known target distance on any axis")
        for item in target_clouds[-self.samples:]:
            fields = item["fields"]
            if not {"x", "y", "z", "rgb"}.issubset(fields):
                errors.append(
                    f"point cloud: missing XYZRGB fields from {sorted(fields)}")
                break
            if any(
                    fields[name][1] != PointField.FLOAT32
                    or fields[name][2] != 1
                    for name in ("x", "y", "z", "rgb")):
                errors.append("point cloud: XYZRGB fields are not scalar FLOAT32")
                break
            if (
                    item["width"] != self.expected_width
                    or item["height"] != self.expected_height
                    or item["height"] <= 1):
                errors.append(
                    "point cloud: cloud is not organized to image dimensions")
                break
            if item["data_size"] != item["row_step"] * item["height"]:
                errors.append(
                    "point cloud: data length does not equal row_step*height")
                break
            if item["frame"] != self.expected_cloud_frame:
                errors.append(
                    f"point cloud: expected frame {self.expected_cloud_frame}, "
                    f"got {item['frame']}")
                break
            if item["valid_fraction"] < self.minimum_valid_fraction:
                errors.append(
                    "point cloud: only "
                    f"{item['valid_fraction']:.1%} XYZ points are finite")
                break
            rgb = item["center_rgb"]
            if (
                    rgb is None
                    or rgb[0] < 60
                    or rgb[0] <= rgb[1] + 40
                    or rgb[0] <= rgb[2] + 40):
                errors.append(
                    "point cloud: center RGB does not contain the red target "
                    f"({rgb})")
                break

        coherent_stamps = self.coherent_target_stamps()
        if len(coherent_stamps) < self.samples:
            errors.append(
                f"timestamp coherence: only {len(coherent_stamps)}/"
                f"{self.samples} target cycles have an exact common stamp")
        else:
            self.validate_registered_frame_contract(coherent_stamps, errors)
            self.validate_target_parallax(coherent_stamps, errors)
            self.validate_cloud_geometry(coherent_stamps, errors)

        return errors

    def summary(self):
        counts = ", ".join(
            f"{topic}={len(items)}"
            for topic, items in self.observations.items())
        return (
            f"{counts}, coherent_target_cycles="
            f"{len(self.coherent_target_stamps())}, {self.parallax_report}")


def main():
    rclpy.init()
    node = DepthGeometryProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
        errors = node.validate()
        if errors:
            node.get_logger().error(
                "RGB-D geometry contract FAILED: " + "; ".join(errors))
            node.get_logger().error("Received: " + node.summary())
            return 1
        node.get_logger().info(
            "RGB-D geometry contract PASSED: " + node.summary())
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
