#!/usr/bin/env python3
"""
Unit tests for the camera adapter's arithmetic and frame handling.

They need sensor_msgs but no running ROS graph: nothing here initializes rclpy.
The cases the reviewer of #49 asked for, adapted from the deleted relay's cloud
input to the adapter's image input, are:

1. a known quaternion: ``TestTransform``;
2. non-finite points: ``TestNonFinitePoints`` (the relay's "left untouched");
3. a padded ``row_step``, now a padded image ``step``: ``TestPaddedRows``;
4. a malformed field or short buffer, now a wrong encoding, a short buffer or
   a size that disagrees with camera_info: ``TestMalformedInput``.

``TestResolveTransform`` covers #56's review note 1: a degenerate transform
must not crash the node, which needs ``CameraAdapter._resolve_transform``
itself, not just ``FramePipeline.set_transform`` (already covered by
``TestTransform``).
"""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointField
from tf2_ros import TransformException

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from camera_adapter import (  # noqa: E402
    AdapterCore,
    CameraAdapter,
    CameraRays,
    FramePipeline,
    StampPairer,
    back_project,
    color_array,
    DepthConditioner,
    condition_depth,
    depth_array,
    depth_image_message,
    pack_cloud,
    rotation_matrix,
)
from cloud_timing import FrameGate, GapSampler  # noqa: E402
from sensor_profiles import SOURCE_PATH, load_sensor_profile  # noqa: E402

OPTICAL_FRAME = "cam_1_color_optical_frame"
CLOUD_FRAME = "cam_1_depth_frame"
FRAME_PERIOD = 0.033
QUARTER_TURN = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
IDENTITY = (0.0, 0.0, 0.0, 1.0)
DECODED = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "u1", 4)])


def stamp(seconds=1.0):
    """Return a Time message."""
    whole = int(seconds)
    return Time(sec=whole, nanosec=round((seconds - whole) * 1e9))


def rows_with_padding(rows, padding):
    """Return image bytes: each row of a (height, row_bytes) array, then padding."""
    padded = np.full((rows.shape[0], rows.shape[1] + padding), 0xEE, dtype=np.uint8)
    padded[:, :rows.shape[1]] = rows
    return padded.tobytes()


def depth_image(depth, padding=0, seconds=1.0, big_endian=False):
    """Return a 32FC1 Image of a (height, width) array of metres."""
    depth = np.asarray(depth)
    dtype = ">f4" if big_endian else "<f4"
    rows = np.ascontiguousarray(depth, dtype=dtype).view(np.uint8)
    message = Image()
    message.header.stamp = stamp(seconds)
    message.header.frame_id = OPTICAL_FRAME
    message.height, message.width = depth.shape
    message.encoding = "32FC1"
    message.is_bigendian = big_endian
    message.step = depth.shape[1] * 4 + padding
    message.data = rows_with_padding(rows, padding)
    return message


def color_image(rgb, padding=0, seconds=1.0, encoding="rgb8"):
    """Return a color Image of a (height, width, 3) uint8 array in R, G, B order."""
    rgb = np.asarray(rgb, dtype=np.uint8)
    pixels = rgb if encoding == "rgb8" else rgb[:, :, ::-1]
    message = Image()
    message.header.stamp = stamp(seconds)
    message.header.frame_id = OPTICAL_FRAME
    message.height, message.width = rgb.shape[:2]
    message.encoding = encoding
    message.step = rgb.shape[1] * 3 + padding
    message.data = rows_with_padding(pixels.reshape(rgb.shape[0], -1), padding)
    return message


def camera_info(width, height, fx=100.0, fy=200.0, cx=1.5, cy=1.0):
    """Return a CameraInfo with the given intrinsics."""
    info = CameraInfo()
    info.width, info.height = width, height
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    return info


def decode(cloud):
    """Return a cloud's points as a structured array."""
    return np.frombuffer(bytes(cloud.data), dtype=DECODED)


def sample_frame(width=4, height=3):
    """Return matching depth and color images with a distinct value at every pixel."""
    v, u = np.mgrid[0:height, 0:width]
    depth = 1.0 + u + 10.0 * v
    rgb = np.stack([10 + u + 20 * v, 100 + u + 20 * v, 200 + u + 20 * v], axis=-1)
    return depth, rgb.astype(np.uint8)


class AlwaysDeliver:
    """A gate that turns every frame into a cloud."""

    @staticmethod
    def deliver(_stamp):
        return True


def pipeline(**arguments):
    """Return a pipeline with a +90 degree z transform and a translation."""
    arguments.setdefault("gate", AlwaysDeliver())
    result = FramePipeline(**arguments)
    result.set_transform((1.0, 2.0, 3.0), QUARTER_TURN)
    return result


def cloud_of(color, depth, info, **arguments):
    """Return the cloud a pipeline builds from a color and a depth image message."""
    target = pipeline(**arguments)
    return target.build_cloud(color, target.condition(depth), info)


class TestBackProjection(unittest.TestCase):
    """A pixel at depth z lies at ((u - cx) / fx * z, (v - cy) / fy * z, z)."""

    def test_every_pixel_lands_where_a_known_intrinsic_matrix_puts_it(self):
        depth, _ = sample_frame()
        info = camera_info(4, 3, fx=100.0, fy=200.0, cx=1.5, cy=1.0)
        points = back_project(depth, CameraRays(info))
        self.assertEqual(points.shape, (12, 3))
        for v in range(3):
            for u in range(4):
                z = depth[v, u]
                expected = ((u - 1.5) / 100.0 * z, (v - 1.0) / 200.0 * z, z)
                np.testing.assert_allclose(points[v * 4 + u], expected, rtol=1e-12)

    def test_the_principal_point_pixel_is_on_the_optical_axis(self):
        info = camera_info(5, 5, fx=50.0, fy=50.0, cx=2.0, cy=2.0)
        points = back_project(np.full((5, 5), 3.0), CameraRays(info))
        np.testing.assert_array_equal(points[12], (0.0, 0.0, 3.0))

    def test_decimation_keeps_every_nth_row_and_column(self):
        depth, _ = sample_frame()
        info = camera_info(4, 3)
        full = back_project(depth, CameraRays(info)).reshape(3, 4, 3)
        kept = back_project(depth, CameraRays(info), decimation=2)
        # Rows 0 and 2, columns 0 and 2: ceil(3 / 2) by ceil(4 / 2).
        np.testing.assert_array_equal(
            kept, full[::2, ::2].reshape(-1, 3))
        self.assertEqual(kept.shape, (4, 3))

    def test_the_rays_are_reused_until_the_intrinsics_change(self):
        rays = CameraRays(camera_info(4, 3))
        self.assertIs(CameraRays.for_info(camera_info(4, 3), rays), rays)
        self.assertIsNot(CameraRays.for_info(camera_info(4, 3, fx=101.0), rays), rays)
        self.assertIsNot(CameraRays.for_info(camera_info(5, 3), rays), rays)
        self.assertIsNotNone(CameraRays.for_info(camera_info(4, 3), None))

    def test_a_depth_image_of_the_wrong_size_is_rejected(self):
        with self.assertRaises(ValueError):
            back_project(np.ones((3, 5)), CameraRays(camera_info(4, 3)))


class TestLayout(unittest.TestCase):
    """The cloud has the physical adapter's layout: 16 bytes, x, y, z and rgb."""

    def setUp(self):
        depth, rgb = sample_frame()
        self.cloud = cloud_of(
            color_image(rgb), depth_image(depth), camera_info(4, 3), strip_nan=False)

    def test_the_fields_are_float32_at_the_physical_offsets(self):
        fields = [(f.name, f.offset, f.datatype, f.count) for f in self.cloud.fields]
        self.assertEqual(fields, [
            ("x", 0, PointField.FLOAT32, 1),
            ("y", 4, PointField.FLOAT32, 1),
            ("z", 8, PointField.FLOAT32, 1),
            ("rgb", 12, PointField.FLOAT32, 1),
        ])

    def test_the_points_are_packed_to_sixteen_bytes(self):
        self.assertEqual(self.cloud.point_step, 16)
        self.assertEqual(self.cloud.row_step, 16 * self.cloud.width)
        self.assertEqual(len(self.cloud.data), self.cloud.row_step * self.cloud.height)
        self.assertFalse(self.cloud.is_bigendian)

    def test_the_header_keeps_the_frame_stamp_and_names_the_depth_frame(self):
        self.assertEqual(self.cloud.header.frame_id, CLOUD_FRAME)
        self.assertEqual(
            (self.cloud.header.stamp.sec, self.cloud.header.stamp.nanosec), (1, 0))

    def test_the_color_is_the_pcl_packed_integer(self):
        _, rgb = sample_frame()
        first = decode(self.cloud)["rgb"][0]
        red, green, blue = (int(channel) for channel in rgb[0, 0])
        # Little-endian bytes: blue, green, red, then zero.
        self.assertEqual(list(first), [blue, green, red, 0])
        as_integer = int(np.frombuffer(first.tobytes(), dtype="<u4")[0])
        self.assertEqual(as_integer, red << 16 | green << 8 | blue)

    def test_a_bgr_image_gives_the_same_colors(self):
        depth, rgb = sample_frame()
        reference = decode(self.cloud)
        cloud = cloud_of(
            color_image(rgb, encoding="bgr8"), depth_image(depth),
            camera_info(4, 3), strip_nan=False)
        np.testing.assert_array_equal(decode(cloud)["rgb"], reference["rgb"])

    def test_the_pixels_come_out_row_by_row(self):
        depth, _ = sample_frame()
        # Under a +90 degree z turn the camera's x axis becomes the cloud's y,
        # so y rises with the column u along each row, then the next row starts.
        np.testing.assert_allclose(
            decode(self.cloud)["y"],
            [2.0 + (u - 1.5) / 100.0 * depth[v, u] for v in range(3) for u in range(4)],
            rtol=1e-6)


class TestStripping(unittest.TestCase):
    """cloud_strip_nan drops non-finite pixels; without it the cloud stays organized."""

    def frame(self):
        depth = np.array([[1.0, np.nan, 2.0], [np.inf, 3.0, -np.inf]])
        rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        return color_image(rgb), depth_image(depth), camera_info(3, 2)

    def test_stripping_keeps_only_finite_points_in_an_unorganized_dense_cloud(self):
        cloud = cloud_of(*self.frame(), strip_nan=True)
        self.assertEqual((cloud.height, cloud.width), (1, 3))
        self.assertTrue(cloud.is_dense)
        self.assertEqual(cloud.row_step, 16 * 3)
        self.assertTrue(np.isfinite(decode(cloud)[["x", "y", "z"]].tolist()).all())

    def test_stripping_keeps_the_colors_of_the_finite_pixels(self):
        cloud = cloud_of(*self.frame(), strip_nan=True)
        rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        kept = [rgb[0, 0], rgb[0, 2], rgb[1, 1]]
        self.assertEqual(
            [list(colour) for colour in decode(cloud)["rgb"]],
            [[int(b), int(g), int(r), 0] for r, g, b in kept])

    def test_without_stripping_the_cloud_is_organized_and_not_dense(self):
        cloud = cloud_of(*self.frame(), strip_nan=False)
        self.assertEqual((cloud.height, cloud.width), (2, 3))
        self.assertFalse(cloud.is_dense)
        finite = np.isfinite(decode(cloud)[["x", "y", "z"]].tolist()).all(axis=1)
        self.assertEqual(finite.tolist(), [True, False, True, False, True, False])

    def test_a_frame_with_nothing_missing_is_dense_either_way(self):
        depth, rgb = sample_frame()
        for strip_nan in (True, False):
            with self.subTest(strip_nan=strip_nan):
                cloud = cloud_of(
                    color_image(rgb), depth_image(depth), camera_info(4, 3),
                    strip_nan=strip_nan)
                self.assertTrue(cloud.is_dense)
                self.assertEqual(cloud.width * cloud.height, 12)

    def test_a_frame_with_nothing_finite_gives_an_empty_cloud(self):
        depth = np.full((2, 3), np.nan)
        cloud = cloud_of(
            color_image(np.zeros((2, 3, 3))), depth_image(depth), camera_info(3, 2),
            strip_nan=True)
        self.assertEqual((cloud.height, cloud.width, len(cloud.data)), (1, 0, 0))
        self.assertTrue(cloud.is_dense)

    def test_decimation_shrinks_the_organized_grid(self):
        depth, rgb = sample_frame()
        cloud = cloud_of(
            color_image(rgb), depth_image(depth), camera_info(4, 3),
            strip_nan=False, decimation=2)
        self.assertEqual((cloud.height, cloud.width), (2, 2))
        self.assertEqual(
            [list(colour) for colour in decode(cloud)["rgb"]],
            [[int(rgb[v, u, 2]), int(rgb[v, u, 1]), int(rgb[v, u, 0]), 0]
             for v in (0, 2) for u in (0, 2)])

    def test_a_bad_decimation_is_rejected(self):
        for decimation in (0, -1, 1.5, True):
            with self.subTest(decimation=decimation):
                with self.assertRaises(ValueError):
                    FramePipeline(AlwaysDeliver(), decimation=decimation)


class TestTransform(unittest.TestCase):
    """The points move into cam_1_depth_frame by the static transform (case 1)."""

    def test_a_quarter_turn_about_z_rotates_x_onto_y(self):
        rotation = rotation_matrix(QUARTER_TURN)
        np.testing.assert_allclose(
            rotation, [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-15)

    def test_a_quaternion_is_normalized_first(self):
        np.testing.assert_allclose(
            rotation_matrix((0.0, 0.0, 2.0, 2.0)), rotation_matrix(QUARTER_TURN),
            atol=1e-15)

    def test_a_rotation_then_a_translation_moves_each_point(self):
        points = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]])
        cloud = pack_cloud(
            points, np.zeros(3, dtype=np.uint32), rotation_matrix(QUARTER_TURN), (1.0, 2.0, 3.0),
            CLOUD_FRAME, stamp(), 1, 3, strip_nan=False)
        decoded = decode(cloud)
        np.testing.assert_allclose(
            np.column_stack([decoded["x"], decoded["y"], decoded["z"]]),
            [[1.0, 3.0, 3.0], [-1.0, 2.0, 3.0], [1.0, 2.0, 6.0]], atol=1e-6)

    def test_the_pipeline_back_projects_then_transforms_every_pixel(self):
        depth, rgb = sample_frame()
        cloud = cloud_of(color_image(rgb), depth_image(depth), camera_info(4, 3))
        decoded = decode(cloud)
        for index, (v, u) in enumerate((v, u) for v in range(3) for u in range(4)):
            z = depth[v, u]
            optical = np.array([(u - 1.5) / 100.0 * z, (v - 1.0) / 200.0 * z, z])
            expected = np.array([-optical[1], optical[0], optical[2]]) + (1.0, 2.0, 3.0)
            np.testing.assert_allclose(
                [decoded["x"][index], decoded["y"][index], decoded["z"][index]],
                expected, rtol=1e-6)

    def test_an_invalid_quaternion_or_translation_is_rejected(self):
        for quaternion in ((0, 0, 0, 0), (0, 0, 0, math.nan), (0, 0, 0, math.inf)):
            with self.subTest(quaternion=quaternion):
                with self.assertRaises(ValueError):
                    rotation_matrix(quaternion)
        target = FramePipeline(AlwaysDeliver())
        with self.assertRaises(ValueError):
            target.set_transform((0.0, math.nan, 0.0), IDENTITY)
        self.assertFalse(target.has_transform)


class FakeLogger:
    """Record throttled log calls by level, standing in for Node.get_logger()."""

    def __init__(self):
        self.info_calls = []
        self.warning_calls = []
        self.error_calls = []

    def info(self, text, **kwargs):
        self.info_calls.append(text)

    def warning(self, text, **kwargs):
        self.warning_calls.append(text)

    def error(self, text, **kwargs):
        self.error_calls.append(text)


class FakeTfBuffer:
    """Stand in for tf2_ros.Buffer: returns a canned transform, or raises."""

    def __init__(self):
        self.transform = None

    def lookup_transform(self, target_frame, source_frame, time):
        del target_frame, source_frame, time
        if self.transform is None:
            raise TransformException("no /tf_static yet")
        return self.transform


def canned_transform(translation, quaternion):
    """Return an object shaped like a TransformStamped, carrying these values."""
    x, y, z = translation
    qx, qy, qz, qw = quaternion
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw)))


class TestResolveTransform(unittest.TestCase):
    """
    The node catches a degenerate transform instead of crashing on it (case 1).

    FramePipeline.set_transform raising is already exercised directly in
    TestTransform; what only the node's own ``_resolve_transform`` can be
    caught doing is surviving that exception, logging it, leaving the
    transform unset, and still publishing the depth image that arrived with it.
    Node.__init__ never runs, so this needs no ROS graph: only the attributes
    ``_resolve_transform`` and ``_on_depth`` themselves read are stubbed.
    """

    @staticmethod
    def node():
        target = CameraAdapter.__new__(CameraAdapter)
        target.target_frame = CLOUD_FRAME
        target.pipeline = FramePipeline(AlwaysDeliver())
        target.tf_buffer = FakeTfBuffer()
        logger = FakeLogger()
        target.get_logger = lambda: logger
        target.depths, target.clouds = [], []
        target.core = AdapterCore(
            target.pipeline, target.depths.append, target.clouds.append,
            lambda text: None, lambda text: None)
        return target, logger

    def test_a_degenerate_quaternion_is_caught_logged_at_error_and_left_unset(self):
        target, logger = self.node()
        target.tf_buffer.transform = canned_transform((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 0.0))
        target._on_depth(depth_image(np.ones((3, 4))))
        self.assertFalse(target.pipeline.has_transform)
        self.assertEqual(len(logger.error_calls), 1)
        self.assertIn("degenerate", logger.error_calls[0])
        self.assertEqual(logger.warning_calls, [])
        # The depth image is still published even though the transform failed.
        self.assertEqual(len(target.depths), 1)

    def test_a_non_finite_translation_is_caught_the_same_way(self):
        target, logger = self.node()
        target.tf_buffer.transform = canned_transform((math.nan, 0.0, 0.0), IDENTITY)
        target._on_depth(depth_image(np.ones((3, 4))))
        self.assertFalse(target.pipeline.has_transform)
        self.assertEqual(len(logger.error_calls), 1)
        self.assertEqual(len(target.depths), 1)

    def test_it_retries_on_the_next_frame_and_recovers_once_tf_is_valid(self):
        target, logger = self.node()
        target.tf_buffer.transform = canned_transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        target._on_depth(depth_image(np.ones((3, 4)), seconds=1.0))
        self.assertFalse(target.pipeline.has_transform)
        self.assertEqual(len(logger.error_calls), 1)
        # A later, valid transform on /tf_static fixes it on the next frame.
        target.tf_buffer.transform = canned_transform((1.0, 2.0, 3.0), IDENTITY)
        target._on_depth(depth_image(np.ones((3, 4)), seconds=1.033))
        self.assertTrue(target.pipeline.has_transform)
        self.assertEqual(len(logger.error_calls), 1)
        self.assertEqual(len(target.depths), 2)

    def test_no_transform_yet_is_still_a_warning_not_an_error(self):
        target, logger = self.node()
        target._on_depth(depth_image(np.ones((3, 4))))
        self.assertFalse(target.pipeline.has_transform)
        self.assertEqual(logger.error_calls, [])
        self.assertEqual(len(logger.warning_calls), 1)


class TestNonFinitePoints(unittest.TestCase):
    """Non-finite points are dropped or kept exactly as they came (case 2)."""

    POINTS = np.array([
        [1.0, 0.0, 0.0],
        [np.nan, 1.0, 2.0],
        [1.0, np.inf, 2.0],
        [np.nan, np.nan, np.nan],
        [0.0, 1.0, 0.0],
    ])

    def pack(self, strip_nan):
        return pack_cloud(
            self.POINTS, np.zeros(5, dtype=np.uint32),
            rotation_matrix(QUARTER_TURN), (1.0, 2.0, 3.0), CLOUD_FRAME, stamp(),
            1, 5, strip_nan)

    def test_stripping_drops_a_point_with_only_x_not_a_number(self):
        cloud = self.pack(strip_nan=True)
        self.assertEqual((cloud.height, cloud.width), (1, 2))
        decoded = decode(cloud)
        np.testing.assert_allclose(
            np.column_stack([decoded["x"], decoded["y"], decoded["z"]]),
            [[1.0, 3.0, 3.0], [0.0, 2.0, 3.0]], atol=1e-6)

    def test_keeping_them_leaves_them_untransformed_and_marks_the_cloud_not_dense(self):
        cloud = self.pack(strip_nan=False)
        self.assertFalse(cloud.is_dense)
        decoded = decode(cloud)
        # x is NaN but y and z are finite: still not a point, so not rotated.
        self.assertTrue(np.isnan(decoded["x"][1]))
        self.assertEqual((decoded["y"][1], decoded["z"][1]), (1.0, 2.0))
        self.assertEqual(decoded["x"][2], 1.0)
        self.assertTrue(np.isinf(decoded["y"][2]))
        self.assertEqual(decoded["z"][2], 2.0)
        self.assertTrue(np.isnan(decoded[["x", "y", "z"]].tolist()[3]).all())
        # The finite ones did move.
        self.assertAlmostEqual(float(decoded["y"][0]), 3.0, places=6)

    def test_the_input_points_are_not_modified(self):
        before = self.POINTS.copy()
        self.pack(strip_nan=False)
        np.testing.assert_array_equal(self.POINTS, before)

    def test_mismatched_points_and_colour_are_rejected(self):
        with self.assertRaises(ValueError):
            pack_cloud(
                self.POINTS, np.zeros(4, dtype=np.uint32), np.eye(3), (0, 0, 0),
                CLOUD_FRAME,
                stamp(), 1, 5, True)
        with self.assertRaises(ValueError):
            pack_cloud(
                self.POINTS, np.zeros(5, dtype=np.uint32), np.eye(3), (0, 0, 0),
                CLOUD_FRAME,
                stamp(), 2, 5, True)


class TestPaddedRows(unittest.TestCase):
    """
    Rows followed by padding are read correctly (case 3).

    The physical adapter's ``metric_depth`` and ``rgb_image`` handle a ``step``
    beyond the row's pixels rather than reject it, so this adapter does too.
    """

    def test_a_padded_depth_image_reads_like_a_packed_one(self):
        depth, _ = sample_frame()
        for padding in (0, 4, 8, 2):
            with self.subTest(padding=padding):
                np.testing.assert_array_equal(
                    depth_array(depth_image(depth, padding=padding)), depth)

    def test_a_padded_color_image_reads_like_a_packed_one(self):
        _, rgb = sample_frame()
        for padding in (0, 1, 3, 7):
            with self.subTest(padding=padding):
                np.testing.assert_array_equal(
                    color_array(color_image(rgb, padding=padding)), rgb)

    def test_padding_does_not_change_the_cloud(self):
        depth, rgb = sample_frame()
        packed = cloud_of(color_image(rgb), depth_image(depth), camera_info(4, 3))
        padded = cloud_of(
            color_image(rgb, padding=5), depth_image(depth, padding=6),
            camera_info(4, 3))
        self.assertEqual(bytes(packed.data), bytes(padded.data))

    def test_the_padding_is_left_out_of_the_published_depth(self):
        depth, _ = sample_frame()
        message = depth_image_message(
            depth_image(depth, padding=8).header,
            depth_array(depth_image(depth, padding=8)))
        self.assertEqual(message.step, 16)
        self.assertEqual(len(message.data), 16 * 3)

    def test_a_row_shorter_than_its_pixels_is_rejected(self):
        depth, rgb = sample_frame()
        short_depth = depth_image(depth)
        short_depth.step -= 1
        short_color = color_image(rgb)
        short_color.step -= 1
        with self.assertRaises(ValueError):
            depth_array(short_depth)
        with self.assertRaises(ValueError):
            color_array(short_color)

    def test_a_big_endian_depth_image_is_read_correctly(self):
        depth, _ = sample_frame()
        np.testing.assert_array_equal(
            depth_array(depth_image(depth, big_endian=True)), depth)


class TestMalformedInput(unittest.TestCase):
    """A malformed frame raises ValueError and is dropped, warned about (case 4)."""

    def setUp(self):
        self.depth, self.rgb = sample_frame()
        self.info = camera_info(4, 3)

    def process(self, color, depth, info=None):
        return cloud_of(color, depth, info or self.info)

    def test_a_wrong_encoding_is_rejected(self):
        depth = depth_image(self.depth)
        depth.encoding = "16UC1"
        with self.assertRaisesRegex(ValueError, "depth encoding"):
            self.process(color_image(self.rgb), depth)
        color = color_image(self.rgb)
        color.encoding = "mono8"
        with self.assertRaisesRegex(ValueError, "color encoding"):
            self.process(color, depth_image(self.depth))

    def test_a_short_buffer_is_rejected(self):
        depth = depth_image(self.depth)
        depth.data = depth.data[:-1]
        with self.assertRaisesRegex(ValueError, "truncated"):
            self.process(color_image(self.rgb), depth)
        color = color_image(self.rgb)
        color.data = color.data[:-1]
        with self.assertRaisesRegex(ValueError, "truncated"):
            self.process(color, depth_image(self.depth))

    def test_an_image_with_no_pixels_is_rejected(self):
        depth = depth_image(self.depth)
        depth.width = 0
        with self.assertRaises(ValueError):
            self.process(color_image(self.rgb), depth)

    def test_images_that_disagree_with_camera_info_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "camera_info"):
            self.process(color_image(self.rgb), depth_image(self.depth), camera_info(5, 3))
        with self.assertRaisesRegex(ValueError, "camera_info"):
            self.process(color_image(self.rgb), depth_image(self.depth), camera_info(4, 4))

    def test_a_malformed_depth_image_is_rejected_on_its_own_without_camera_info(self):
        target = pipeline()
        for name, edit in (
                ("encoding", lambda depth: setattr(depth, "encoding", "16UC1")),
                ("short buffer", lambda depth: setattr(depth, "data", depth.data[:-1])),
                ("no pixels", lambda depth: setattr(depth, "width", 0)),
                ("short row", lambda depth: setattr(depth, "step", depth.step - 1))):
            with self.subTest(name=name):
                depth = depth_image(self.depth)
                edit(depth)
                with self.assertRaises(ValueError):
                    target.condition(depth)

    def test_a_size_that_disagrees_with_camera_info_is_rejected_only_for_the_cloud(self):
        target = pipeline()
        conditioned = target.condition(depth_image(self.depth))
        self.assertEqual(conditioned.pixels.shape, (3, 4))
        with self.assertRaisesRegex(ValueError, "camera_info"):
            target.build_cloud(color_image(self.rgb), conditioned, camera_info(5, 3))
        # The conditioned depth is intact, and still builds a cloud.
        self.assertIsNotNone(
            target.build_cloud(color_image(self.rgb), conditioned, self.info))

    def test_color_and_depth_of_different_sizes_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "differ in size"):
            self.process(
                color_image(self.rgb[:, :3]), depth_image(self.depth))

    def test_invalid_intrinsics_are_rejected(self):
        for name, edit in (
                ("no focal length", lambda info: info.k.__setitem__(0, 0.0)),
                ("negative focal length", lambda info: info.k.__setitem__(4, -1.0)),
                ("not a number", lambda info: info.k.__setitem__(2, math.nan)),
                ("no size", lambda info: setattr(info, "width", 0))):
            with self.subTest(name=name):
                info = camera_info(4, 3)
                edit(info)
                with self.assertRaises(ValueError):
                    self.process(
                        color_image(self.rgb), depth_image(self.depth), info)


class TestDepthConditioning(unittest.TestCase):
    """The step-7 seam runs once on every depth image, and feeds both outputs."""

    def test_it_runs_on_every_depth_image_but_a_cloud_is_built_only_for_delivered_ones(self):
        calls = []

        def condition(depth, _stamp):
            calls.append(depth)
            return depth * 2.0

        depth, rgb = sample_frame()
        gate = FrameGate(GapSampler({3: 1.0}, 1), FRAME_PERIOD)
        target = pipeline(gate=gate, condition=condition, strip_nan=False)
        clouds, conditioned = [], []
        for frame in range(7):
            seconds = 1.0 + frame * FRAME_PERIOD
            frame_depth = target.condition(depth_image(depth, seconds=seconds))
            conditioned.append(frame_depth.pixels)
            clouds.append(target.build_cloud(
                color_image(rgb, seconds=seconds), frame_depth, camera_info(4, 3)))
        self.assertEqual(len(calls), 7)
        self.assertEqual(
            [cloud is not None for cloud in clouds],
            [True, False, False, True, False, False, True])
        # The cloud is built from the conditioned frame: twice the rendered depth.
        for frame_depth in conditioned:
            np.testing.assert_array_equal(frame_depth, depth * 2.0)
        # Under the +90 degree z turn the depth (optical z) stays the z axis.
        np.testing.assert_allclose(
            decode(clouds[0])["z"], (depth * 2.0).ravel() + 3.0, rtol=1e-6)


DEPTH_SHAPE = (240, 320)
PHYSICAL = load_sensor_profile(SOURCE_PATH, "depth", "physical")
# The values under test, read from the profile file. The expectations below
# are worked out from them with math.pow, never with the adapter's arithmetic.
SCALE = 1.0 + PHYSICAL["scale_error"]


def physical_conditioner(seed=5, **changes):
    """Return a conditioner built from the physical depth profile."""
    values = dict(
        scale_error=PHYSICAL["scale_error"],
        noise_floor_m=PHYSICAL["noise_sigma_floor_m"],
        noise_coefficient=PHYSICAL["noise_sigma_coefficient"],
        noise_exponent=PHYSICAL["noise_sigma_exponent"],
        min_range_m=PHYSICAL["min_range_m"],
        seed=seed)
    values.update(changes)
    return DepthConditioner(**values)


def expected_sigma(rendered):
    """Return sigma for a rendered depth: the scaled depth into the noise law."""
    reported = rendered * SCALE
    return max(
        PHYSICAL["noise_sigma_floor_m"],
        PHYSICAL["noise_sigma_coefficient"]
        * math.pow(reported, PHYSICAL["noise_sigma_exponent"]))


class TestIdealDepth(unittest.TestCase):
    """The ideal profile publishes every finite pixel bit for bit, and NaN for the rest."""

    @staticmethod
    def rendered():
        generator = np.random.default_rng(11)
        depth = generator.uniform(0.05, 8.0, DEPTH_SHAPE).astype(np.float32)
        depth[0, :5] = [np.inf, -np.inf, np.nan, 0.0, -0.5]
        depth[1, :3] = np.finfo(np.float32).tiny / 4  # a subnormal
        return depth

    def check(self, conditioned, rendered):
        self.assertEqual(conditioned.dtype, np.float32)
        finite = np.isfinite(rendered)
        np.testing.assert_array_equal(
            conditioned.view(np.uint32)[finite], rendered.view(np.uint32)[finite])
        self.assertTrue(np.all(np.isnan(conditioned[~finite])))
        self.assertFalse(np.any(np.isinf(conditioned)))

    def test_finite_pixels_are_unchanged_and_every_other_pixel_is_nan(self):
        rendered = self.rendered()
        self.check(condition_depth(rendered), rendered)

    def test_an_unset_conditioner_is_that_ideal_path_and_draws_nothing(self):
        conditioner = DepthConditioner()

        def refuse(*_):
            raise AssertionError("the ideal path must not draw noise")

        conditioner._noise = refuse
        rendered = self.rendered()
        self.check(conditioner(rendered, 123), rendered)

    def test_the_input_is_left_alone(self):
        rendered = self.rendered()
        before = rendered.copy()
        condition_depth(rendered)
        physical_conditioner()(rendered, 1)
        np.testing.assert_array_equal(rendered.view(np.uint32), before.view(np.uint32))


class TestPhysicalDepth(unittest.TestCase):
    """The physical profile's scale error, noise and minimum range."""

    @staticmethod
    def flat(distance, height=DEPTH_SHAPE[0], width=DEPTH_SHAPE[1]):
        return np.full((height, width), distance, dtype=np.float32)

    def test_the_scale_error_alone_scales_every_distance(self):
        conditioner = physical_conditioner(noise_floor_m=0.0, noise_coefficient=0.0)
        for distance in (0.7, 1.5, 4.0, 7.9):
            with self.subTest(distance=distance):
                published = conditioner(self.flat(distance), 1)
                np.testing.assert_allclose(
                    published, distance * SCALE, rtol=2e-7, atol=0.0)

    def test_the_mean_and_deviation_follow_the_law_at_each_distance(self):
        conditioner = physical_conditioner()
        frames = 4
        for distance in (0.8, 1.05, 1.5, 2.5, 3.6, 5.0):
            with self.subTest(distance=distance):
                published = np.concatenate([
                    conditioner(self.flat(distance), 1000 + frame).ravel()
                    for frame in range(frames)]).astype(np.float64)
                count = published.size
                reported = distance * SCALE
                sigma = expected_sigma(distance)
                # Five standard errors of the mean and of a standard deviation.
                self.assertAlmostEqual(
                    published.mean(), reported, delta=5.0 * sigma / math.sqrt(count))
                self.assertAlmostEqual(
                    published.std() / sigma, 1.0, delta=5.0 / math.sqrt(2.0 * count))

    def test_the_noise_is_the_floor_up_close_and_grows_as_the_power_law_beyond(self):
        conditioner = physical_conditioner()
        spread = {
            distance: float(conditioner(self.flat(distance), 7).astype(np.float64).std())
            for distance in (0.7, 1.0, 3.0, 6.0)}
        # Under the floor's knee (about 1.02 m reported) the deviation is 2 mm.
        for distance in (0.7, 1.0):
            self.assertAlmostEqual(spread[distance], 0.002, delta=0.0002)
        # Beyond it, doubling the distance multiplies sigma by 2 ** 2.36 = 5.13.
        self.assertAlmostEqual(spread[6.0] / spread[3.0], 2.0 ** 2.36, delta=0.15)

    def test_nothing_below_the_minimum_range_is_ever_published(self):
        generator = np.random.default_rng(2)
        rendered = generator.uniform(0.3, 8.0, DEPTH_SHAPE).astype(np.float32)
        minimum = PHYSICAL["min_range_m"]
        conditioner = physical_conditioner()
        for stamp_ns in range(20):
            published = conditioner(rendered, stamp_ns)
            self.assertGreaterEqual(float(np.nanmin(published)), minimum)
            self.assertFalse(np.any(np.isinf(published)))

    def test_pixels_well_under_the_cutoff_are_nan_and_well_over_it_are_not(self):
        minimum = PHYSICAL["min_range_m"]
        boundary = minimum / SCALE
        # Six sigma of the floor is 12 mm: rendered depths that far off the
        # boundary can only fall on one side.
        margin = 6.0 * PHYSICAL["noise_sigma_floor_m"] / SCALE
        conditioner = physical_conditioner()
        under = conditioner(self.flat(boundary - margin - 0.001), 4)
        over = conditioner(self.flat(boundary + margin + 0.001), 4)
        self.assertTrue(np.all(np.isnan(under)))
        self.assertFalse(np.any(np.isnan(over)))

    def test_at_the_cutoff_about_half_the_pixels_survive(self):
        boundary = PHYSICAL["min_range_m"] / SCALE
        published = physical_conditioner()(self.flat(boundary), 4)
        survived = float(np.mean(np.isfinite(published)))
        # Half, within five standard errors of a fair coin over 76,800 pixels.
        self.assertAlmostEqual(survived, 0.5, delta=5 * 0.5 / math.sqrt(published.size))

    def test_no_return_pixels_are_nan_not_infinite(self):
        rendered = self.flat(2.0)
        rendered[0, :3] = [np.inf, -np.inf, np.nan]
        published = physical_conditioner()(rendered, 1)
        self.assertTrue(np.all(np.isnan(published[0, :3])))
        self.assertFalse(np.any(np.isinf(published)))
        self.assertTrue(np.all(np.isfinite(published[1:])))

    def test_the_output_is_float32(self):
        self.assertEqual(physical_conditioner()(self.flat(2.0), 1).dtype, np.float32)
        self.assertEqual(
            physical_conditioner()(self.flat(2.0).astype(np.float64), 1).dtype, np.float32)

    def test_neighbouring_pixels_and_frames_are_uncorrelated(self):
        conditioner = physical_conditioner()
        first = conditioner(self.flat(3.0), 1).astype(np.float64)
        second = conditioner(self.flat(3.0), 2).astype(np.float64)
        first -= first.mean()
        second -= second.mean()
        limit = 5.0 / math.sqrt(first.size)
        horizontal = float(np.mean(first[:, 1:] * first[:, :-1]) / first.var())
        vertical = float(np.mean(first[1:] * first[:-1]) / first.var())
        across = float(np.mean(first * second) / math.sqrt(first.var() * second.var()))
        for name, correlation in (
                ("horizontal", horizontal), ("vertical", vertical), ("frames", across)):
            with self.subTest(neighbour=name):
                self.assertLess(abs(correlation), limit)

    def test_invalid_parameters_are_rejected(self):
        for name, value in (
                ("scale_error", -1.0), ("scale_error", math.nan),
                ("noise_floor_m", -0.1), ("noise_coefficient", -1.0),
                ("noise_exponent", 0.0), ("min_range_m", -0.6),
                ("min_range_m", math.inf)):
            with self.subTest(name=name, value=value):
                with self.assertRaises(ValueError):
                    physical_conditioner(**{name: value})


class TestDepthNoiseStream(unittest.TestCase):
    """
    A frame's noise comes from (seed, stamp) alone, in a stream of its own.

    numpy does not promise the same random stream across versions, so nothing
    here pins a noise value: only reproducibility within one process. The gap
    sampler uses Python's ``random``, so its sequence is pinned below.
    """

    def setUp(self):
        self.depth = np.full(DEPTH_SHAPE, 3.0, dtype=np.float32)

    def test_the_same_seed_and_stamp_give_the_same_frame_bit_for_bit(self):
        first = physical_conditioner(seed=9)(self.depth, 5_000_000_000)
        again = physical_conditioner(seed=9)(self.depth, 5_000_000_000)
        np.testing.assert_array_equal(first.view(np.uint32), again.view(np.uint32))

    def test_another_stamp_or_seed_gives_other_noise(self):
        base = physical_conditioner(seed=9)(self.depth, 5_000_000_000)
        for other in (
                physical_conditioner(seed=9)(self.depth, 5_033_000_000),
                physical_conditioner(seed=10)(self.depth, 5_000_000_000)):
            self.assertFalse(np.array_equal(base, other))

    def test_a_dropped_frame_does_not_shift_the_noise_of_the_next(self):
        stamps = [1_000_000_000, 1_033_000_000, 1_066_000_000]
        every = physical_conditioner(seed=3)
        with_gap = physical_conditioner(seed=3)
        kept = [every(self.depth, stamp) for stamp in stamps]
        skipped = [with_gap(self.depth, stamps[0]), with_gap(self.depth, stamps[2])]
        np.testing.assert_array_equal(kept[0], skipped[0])
        np.testing.assert_array_equal(kept[2], skipped[1])

    def test_a_pixels_noise_does_not_depend_on_which_others_are_valid(self):
        holed = self.depth.copy()
        holed[:100] = np.inf
        conditioner = physical_conditioner(seed=4)
        full = conditioner(self.depth, 77)
        partial = conditioner(holed, 77)
        np.testing.assert_array_equal(partial[100:], full[100:])
        self.assertTrue(np.all(np.isnan(partial[:100])))

    def test_the_cloud_gap_sequence_for_a_seed_is_what_main_produced(self):
        """The gap sampler is untouched: these are main's draws, captured at 7da6469."""
        table = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")["gap_frames"]
        pinned = {
            0: [6, 4, 2, 1, 2, 2, 5, 1, 2, 3, 8, 2, 1, 4, 3, 1, 8, 17, 5, 8,
                1, 4, 8, 4, 2, 1, 2, 3, 9, 13, 2, 7, 1, 5, 3, 1, 4, 2, 6, 3],
            1: [1, 6, 5, 1, 2, 2, 3, 5, 1, 1, 6, 2, 5, 1, 2, 4, 1, 11, 8, 1,
                1, 3, 10, 2, 1, 2, 1, 1, 2, 2, 1, 1, 1, 2, 1, 1, 6, 3, 3, 1],
            20260926: [1, 1, 1, 2, 1, 1, 2, 5, 6, 1, 4, 4, 4, 4, 3, 1, 3, 2, 1, 7,
                       1, 1, 1, 2, 2, 7, 8, 11, 1, 4, 5, 1, 2, 2, 5, 1, 4, 3, 2, 1],
        }
        for seed, gaps in pinned.items():
            with self.subTest(seed=seed):
                sampler = GapSampler(table, seed)
                self.assertEqual([sampler.next_gap() for _ in gaps], gaps)

    def test_conditioning_leaves_the_delivered_frames_of_a_seed_unchanged(self):
        """The same seed feeds both; the frames that become clouds are main's."""
        table = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")["gap_frames"]
        pinned = [
            0, 1, 2, 5, 6, 9, 11, 12, 14, 15, 17, 18, 19, 21, 27, 28, 29, 32, 43, 46,
            48, 62, 63, 69, 70, 71, 72, 73, 79, 80, 83, 86, 88, 91, 92, 93, 94, 98,
            100, 101, 104, 106, 107, 112, 116, 117, 120, 123, 130, 134, 135, 150,
            151, 153, 157, 158, 160, 161, 164, 169, 172, 179, 180, 184, 187, 190,
            192, 198, 209, 211, 214, 215, 219, 222, 242, 248, 249, 251, 254, 255,
            257, 258, 259, 260, 265, 266, 267, 269, 276, 277, 279, 282, 289, 295]
        depth, rgb = sample_frame()
        target = FramePipeline(
            FrameGate(GapSampler(table, 7), FRAME_PERIOD),
            condition=physical_conditioner(seed=7))
        target.set_transform((0.0, 0.0, 0.0), IDENTITY)
        delivered = []
        for frame in range(300):
            seconds = 1.0 + frame * FRAME_PERIOD
            conditioned = target.condition(depth_image(depth, seconds=seconds))
            cloud = target.build_cloud(
                color_image(rgb, seconds=seconds), conditioned, camera_info(4, 3))
            if cloud is not None:
                delivered.append(frame)
        self.assertEqual(delivered, pinned)


class TestOneConditionedFrame(unittest.TestCase):
    """The public depth image and the cloud come from one conditioned array."""

    def test_the_cloud_holds_exactly_the_published_finite_pixels(self):
        depths, clouds, warnings, errors = [], [], [], []
        conditioner = physical_conditioner(seed=1)
        calls = []

        def counting(depth, stamp_nanoseconds):
            calls.append(stamp_nanoseconds)
            return conditioner(depth, stamp_nanoseconds)

        target = FramePipeline(AlwaysDeliver(), condition=counting)
        target.set_transform((0.0, 0.0, 0.0), IDENTITY)
        core = AdapterCore(
            target, depths.append, clouds.append, warnings.append, errors.append)
        core.on_camera_info(camera_info(6, 4, fx=50.0, fy=50.0, cx=2.5, cy=1.5))
        generator = np.random.default_rng(6)
        rendered = generator.uniform(0.3, 6.0, (4, 6)).astype(np.float32)
        rendered[0, 0] = np.inf
        rgb = np.zeros((4, 6, 3), np.uint8)
        for index in range(3):
            seconds = 1.0 + index * FRAME_PERIOD
            core.on_color(color_image(rgb, seconds=seconds))
            core.on_depth(depth_image(rendered, seconds=seconds))
        self.assertEqual(len(calls), 3)
        self.assertEqual((len(depths), len(clouds)), (3, 3))
        for message, cloud in zip(depths, clouds):
            published = depth_array(message)
            self.assertFalse(np.array_equal(published, rendered))
            finite = np.isfinite(published).ravel()
            # Stripped of non-finite points, in row order: the finite pixels'
            # depths are the cloud's z, unchanged, so no second draw was made.
            np.testing.assert_array_equal(
                decode(cloud)["z"], published.ravel()[finite])
        self.assertEqual(warnings + errors, [])


class TestStampPairer(unittest.TestCase):
    """Color and depth are matched by their exact stamp."""

    @staticmethod
    def image(seconds):
        message = Image()
        message.header.stamp = stamp(seconds)
        return message

    def test_a_pair_completes_in_either_order(self):
        for first, second in (("color", "depth"), ("depth", "color")):
            with self.subTest(first=first):
                pairer = StampPairer()
                early, late = self.image(1.0), self.image(1.0)
                self.assertIsNone(pairer.add(first, early))
                color, depth = pairer.add(second, late)
                self.assertIs(color, early if first == "color" else late)
                self.assertIs(depth, early if first == "depth" else late)

    def test_stamps_that_differ_at_all_do_not_pair(self):
        pairer = StampPairer()
        self.assertIsNone(pairer.add("color", self.image(1.0)))
        self.assertIsNone(pairer.add("depth", self.image(1.000000001)))

    def test_a_completed_pair_discards_older_stragglers(self):
        pairer = StampPairer()
        pairer.add("color", self.image(1.0))
        pairer.add("depth", self.image(2.0))
        pairer.add("color", self.image(3.0))
        self.assertIsNotNone(pairer.add("depth", self.image(3.0)))
        self.assertEqual(pairer.discarded, 2)

    def test_an_image_that_never_pairs_is_evicted_at_capacity(self):
        pairer = StampPairer(capacity=3)
        for index in range(5):
            pairer.add("color", self.image(float(index + 1)))
        self.assertEqual(pairer.discarded, 2)


class TestAdapterCore(unittest.TestCase):
    """The core publishes each depth image as it arrives, and a cloud when it is delivered."""

    def setUp(self):
        self.depths, self.clouds, self.warnings, self.errors = [], [], [], []
        gate = FrameGate(GapSampler({2: 1.0}, 1), FRAME_PERIOD)
        self.core = AdapterCore(
            pipeline(gate=gate), self.depths.append, self.clouds.append,
            self.warnings.append, self.errors.append)
        self.depth, self.rgb = sample_frame()

    def frame(self, index, color=True, depth=True):
        seconds = 1.0 + index * FRAME_PERIOD
        if color:
            self.core.on_color(color_image(self.rgb, seconds=seconds))
        if depth:
            self.core.on_depth(depth_image(self.depth, seconds=seconds))

    def outputs(self):
        return len(self.depths), len(self.clouds)

    def test_every_depth_image_is_published_and_every_other_frame_has_a_cloud(self):
        self.core.on_camera_info(camera_info(4, 3))
        for index in range(5):
            self.frame(index)
        self.assertEqual(len(self.depths), 5)
        self.assertEqual(len(self.clouds), 3)
        self.assertEqual(self.warnings, [])

    def test_the_published_depth_is_packed_and_keeps_the_frame_header(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0)
        message = self.depths[0]
        self.assertEqual(message.header.frame_id, OPTICAL_FRAME)
        self.assertEqual(message.encoding, "32FC1")
        self.assertEqual(message.step, 16)
        np.testing.assert_array_equal(depth_array(message), self.depth)

    def test_the_cloud_keeps_its_frames_stamp(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(2)
        self.frame(3)
        self.assertEqual(
            [cloud.header.stamp.nanosec for cloud in self.clouds],
            [stamp(1.0 + 2 * FRAME_PERIOD).nanosec])

    def test_a_depth_image_is_published_with_no_color_partner(self):
        self.core.on_camera_info(camera_info(4, 3))
        for index in range(4):
            self.frame(index, color=False)
            self.assertEqual(self.outputs(), (index + 1, 0))
        np.testing.assert_array_equal(depth_array(self.depths[0]), self.depth)
        self.assertEqual(self.warnings, [])

    def test_a_depth_image_is_published_before_the_first_camera_info(self):
        self.frame(0, color=False)
        self.assertEqual(self.outputs(), (1, 0))
        self.assertEqual(self.warnings, [])
        # Its color partner arrives, and still no camera_info: there is no cloud
        # to build, so the frame says so, and the depth is not published again.
        self.frame(0, depth=False)
        self.assertEqual(self.outputs(), (1, 0))
        self.assertIn("camera_info", self.warnings[0])
        # With camera_info the next frame builds its cloud. A frame that had no
        # camera_info did not use up the gate's first delivery.
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(1)
        self.assertEqual(self.outputs(), (2, 1))

    def test_the_depth_is_published_when_it_arrives_whichever_image_comes_first(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0, depth=False)
        self.assertEqual(self.outputs(), (0, 0))
        self.frame(0, color=False)
        self.assertEqual(self.outputs(), (1, 1))
        self.frame(1, color=False)
        self.assertEqual(self.outputs(), (2, 1))
        self.frame(1, depth=False)
        self.assertEqual(self.outputs(), (2, 1))

    def test_the_cloud_is_built_from_the_array_that_was_published(self):
        noise = np.random.default_rng(3)
        calls = []

        def condition(depth, _stamp):
            calls.append(depth)
            return depth + noise.normal(0.0, 0.01, depth.shape)

        target = FramePipeline(AlwaysDeliver(), strip_nan=False, condition=condition)
        # An identity transform leaves each cloud z the very value of its pixel.
        target.set_transform((0.0, 0.0, 0.0), IDENTITY)
        core = AdapterCore(
            target, self.depths.append, self.clouds.append, self.warnings.append,
            self.errors.append)
        core.on_camera_info(camera_info(4, 3))
        for index in range(3):
            seconds = 1.0 + index * FRAME_PERIOD
            core.on_color(color_image(self.rgb, seconds=seconds))
            core.on_depth(depth_image(self.depth, seconds=seconds))
        # Conditioned once per depth image, not once per output: a second run
        # would draw different noise for the cloud than for the image.
        self.assertEqual(len(calls), 3)
        self.assertEqual(self.outputs(), (3, 3))
        for depth_message, cloud in zip(self.depths, self.clouds):
            published = depth_array(depth_message)
            self.assertFalse(np.array_equal(published, self.depth))
            np.testing.assert_array_equal(decode(cloud)["z"], published.ravel())
        self.assertFalse(np.array_equal(
            depth_array(self.depths[0]), depth_array(self.depths[1])))

    def test_a_camera_info_size_mismatch_drops_the_cloud_but_not_the_depth(self):
        self.core.on_camera_info(camera_info(5, 3))
        self.frame(0)
        self.assertEqual(self.outputs(), (1, 0))
        np.testing.assert_array_equal(depth_array(self.depths[0]), self.depth)
        self.assertEqual(self.warnings, [])
        self.assertEqual(len(self.errors), 1)
        self.assertIn("camera_info", self.errors[0])
        # The mismatched frame did not use up the gate's first delivery.
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(1)
        self.assertEqual(self.outputs(), (2, 1))

    def test_a_malformed_color_image_drops_the_cloud_but_not_the_depth(self):
        self.core.on_camera_info(camera_info(4, 3))
        bad = color_image(self.rgb)
        bad.encoding = "mono8"
        self.core.on_color(bad)
        self.core.on_depth(depth_image(self.depth))
        self.assertEqual(self.outputs(), (1, 0))
        self.assertEqual(self.warnings, [])
        self.assertEqual(len(self.errors), 1)
        self.assertIn("Dropping the cloud", self.errors[0])
        self.frame(1)
        self.assertEqual(self.outputs(), (2, 1))

    def test_an_invalid_depth_image_is_dropped_whole_and_logged_at_error(self):
        self.core.on_camera_info(camera_info(4, 3))
        bad = depth_image(self.depth)
        bad.data = bad.data[:-4]
        self.core.on_color(color_image(self.rgb))
        self.core.on_depth(bad)
        self.assertEqual(self.outputs(), (0, 0))
        self.assertEqual(self.warnings, [])
        self.assertEqual(len(self.errors), 1)
        self.assertIn("Dropping invalid depth image", self.errors[0])
        # The next frame is fine and goes through.
        self.frame(1)
        self.assertEqual(self.outputs(), (1, 1))

    def test_a_color_image_without_a_partner_makes_nothing(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0, depth=False)
        self.assertEqual(self.outputs(), (0, 0))

    def test_stragglers_are_reported_and_counted(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0, depth=False)
        self.frame(1)
        self.assertEqual(len(self.depths), 1)
        self.assertTrue(any("never found" in text for text in self.warnings))
        self.assertEqual(self.core.discarded, 1)

    def test_it_conditions_a_depth_image_that_never_finds_a_partner(self):
        calls = []

        def condition(depth, _stamp):
            calls.append(depth)
            return depth * 2.0

        core = AdapterCore(
            pipeline(condition=condition), self.depths.append, self.clouds.append,
            self.warnings.append, self.errors.append)
        core.on_depth(depth_image(self.depth))
        self.assertEqual(len(calls), 1)
        np.testing.assert_array_equal(depth_array(self.depths[0]), self.depth * 2.0)


if __name__ == "__main__":
    unittest.main()
