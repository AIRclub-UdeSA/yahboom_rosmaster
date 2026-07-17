#!/usr/bin/env python3
"""Validate RGB-D response against a known red box in the empty world."""

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


COLOR_TOPIC = "/cam_1/color/image_raw"
DEPTH_TOPIC = "/cam_1/depth/image_raw"
COLOR_INFO_TOPIC = "/cam_1/color/camera_info"
DEPTH_INFO_TOPIC = "/cam_1/depth/camera_info"
POINTS_TOPIC = "/cam_1/depth/color/points"
EXPECTED_IMAGE_FRAME = "cam_1_depth_optical_frame"
EXPECTED_CLOUD_FRAME = "cam_1_depth_frame"
EXPECTED_WIDTH = 424
EXPECTED_HEIGHT = 240
DEPTH_NEAR = 0.05
DEPTH_FAR = 1.5
OFF_AXIS_COLUMN_OFFSET = 40
OFF_AXIS_ROW_OFFSET = 20


class DepthGeometryProbe(Node):
    """Collect synchronized camera samples and validate known scene geometry."""

    def __init__(self):
        super().__init__("depth_geometry_probe")
        self.declare_parameter("timeout", 45.0)
        self.declare_parameter("samples", 10)
        self.declare_parameter("expected_depth", 0.695)
        self.declare_parameter("depth_tolerance", 0.03)
        self.declare_parameter("target_red_pixels", 500)
        self.declare_parameter("minimum_valid_fraction", 0.02)

        self.timeout = float(self.get_parameter("timeout").value)
        self.samples = max(10, int(self.get_parameter("samples").value))
        self.expected_depth = float(
            self.get_parameter("expected_depth").value)
        self.depth_tolerance = float(
            self.get_parameter("depth_tolerance").value)
        self.target_red_pixels = int(
            self.get_parameter("target_red_pixels").value)
        self.minimum_valid_fraction = float(
            self.get_parameter("minimum_valid_fraction").value)

        self.observations = {
            COLOR_TOPIC: [],
            DEPTH_TOPIC: [],
            COLOR_INFO_TOPIC: [],
            DEPTH_INFO_TOPIC: [],
            POINTS_TOPIC: [],
        }
        self.capture_errors = []
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
        """Retain a bounded sample history for final diagnostics."""
        if len(self.observations[topic]) < 120:
            self.observations[topic].append(observation)

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
            if message.encoding == "rgb8":
                image = self.image_array(message, 3, np.uint8).astype(
                    np.int16, copy=False)
                red_mask = (
                    (image[:, :, 0] > 60)
                    & (image[:, :, 0] > image[:, :, 1] + 40)
                    & (image[:, :, 0] > image[:, :, 2] + 40)
                )
                red_pixels = int(np.count_nonzero(red_mask))
            self.append(COLOR_TOPIC, {
                "stamp": self.stamp_ns(message),
                "frame": message.header.frame_id,
                "encoding": message.encoding,
                "width": int(message.width),
                "height": int(message.height),
                "data_size": len(message.data),
                "expected_data_size": int(message.step * message.height),
                "red_pixels": red_pixels,
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
        if len(self.observations[COLOR_TOPIC]) < self.samples + 3:
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
                    EXPECTED_WIDTH, EXPECTED_HEIGHT):
                errors.append(
                    f"{topic}: expected {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}, "
                    f"got {item['width']}x{item['height']}")
                break
            if item["data_size"] != item["expected_data_size"]:
                errors.append(f"{topic}: data length does not equal step*height")
                break
            if item["frame"] != EXPECTED_IMAGE_FRAME:
                errors.append(
                    f"{topic}: expected frame {EXPECTED_IMAGE_FRAME}, "
                    f"got {item['frame']}")
                break

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

        baseline = colors[:3]
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
                    or item["valid_min"] < DEPTH_NEAR - 0.01
                    or item["valid_max"] > DEPTH_FAR + 0.01):
                errors.append(
                    "depth validity: finite values fall outside configured "
                    f"clip range ({item['valid_min']}, {item['valid_max']})")
                break

        for topic, info_samples in (
                (COLOR_INFO_TOPIC, color_info),
                (DEPTH_INFO_TOPIC, depth_info)):
            for item in info_samples:
                if (item["width"], item["height"]) != (
                        EXPECTED_WIDTH, EXPECTED_HEIGHT):
                    errors.append(f"{topic}: dimensions do not match images")
                    break
                if item["frame"] != EXPECTED_IMAGE_FRAME:
                    errors.append(
                        f"{topic}: expected frame {EXPECTED_IMAGE_FRAME}, "
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
                    item["width"] != EXPECTED_WIDTH
                    or item["height"] != EXPECTED_HEIGHT
                    or item["height"] <= 1):
                errors.append(
                    "point cloud: cloud is not organized to image dimensions")
                break
            if item["data_size"] != item["row_step"] * item["height"]:
                errors.append(
                    "point cloud: data length does not equal row_step*height")
                break
            if item["frame"] != EXPECTED_CLOUD_FRAME:
                errors.append(
                    f"point cloud: expected frame {EXPECTED_CLOUD_FRAME}, "
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
            depth_by_stamp = {item["stamp"]: item for item in depths}
            cloud_by_stamp = {item["stamp"]: item for item in clouds}
            depth_frame_matches = 0
            off_axis_matches = 0
            first_mismatch = None
            for stamp in coherent_stamps[-self.samples:]:
                depth_sample = depth_by_stamp[stamp]
                cloud_sample = cloud_by_stamp[stamp]
                depth = depth_sample["center_depth"]
                xyz = cloud_sample["center_xyz"]
                if (
                        xyz is not None
                        and math.isclose(xyz[0], depth, abs_tol=0.01)
                        and abs(xyz[1]) < 0.01
                        and abs(xyz[2]) < 0.01):
                    depth_frame_matches += 1
                elif first_mismatch is None:
                    first_mismatch = (depth, xyz)

                off_depth = depth_sample["off_axis_depth"]
                off_xyz = cloud_sample["off_axis_xyz"]
                info = next(
                    item for item in depth_info if item["stamp"] == stamp)
                expected_y = -(
                    OFF_AXIS_COLUMN_OFFSET * off_depth / info["k"][0])
                expected_z = -(
                    OFF_AXIS_ROW_OFFSET * off_depth / info["k"][4])
                if (
                        off_xyz is not None
                        and math.isfinite(off_depth)
                        and math.isclose(
                            off_depth, self.expected_depth,
                            abs_tol=self.depth_tolerance)
                        and math.isclose(
                            off_xyz[0], off_depth, abs_tol=0.01)
                        and math.isclose(
                            off_xyz[1], expected_y, abs_tol=0.01)
                        and math.isclose(
                            off_xyz[2], expected_z, abs_tol=0.01)):
                    off_axis_matches += 1
            if depth_frame_matches < self.samples:
                errors.append(
                    "point cloud depth-frame geometry: "
                    f"{depth_frame_matches}/{self.samples} center points use "
                    "ROS depth-frame +X-forward coordinates; first depth/XYZ "
                    f"mismatch was {first_mismatch}")
            if off_axis_matches < self.samples:
                first_stamp = coherent_stamps[-self.samples]
                first_depth = depth_by_stamp[first_stamp]["off_axis_depth"]
                first_xyz = cloud_by_stamp[first_stamp]["off_axis_xyz"]
                errors.append(
                    "point cloud off-axis geometry: "
                    f"{off_axis_matches}/{self.samples} samples match +X "
                    "forward, +Y left, +Z up signs at the lower-right image "
                    f"pixel; first depth/XYZ was {first_depth}/{first_xyz}")

        return errors

    def summary(self):
        counts = ", ".join(
            f"{topic}={len(items)}"
            for topic, items in self.observations.items())
        return (
            f"{counts}, coherent_target_cycles="
            f"{len(self.coherent_target_stamps())}")


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
