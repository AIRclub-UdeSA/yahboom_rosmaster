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
"""

import math
from pathlib import Path
import sys
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointField

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from camera_adapter import (  # noqa: E402
    AdapterCore,
    CameraRays,
    FramePipeline,
    StampPairer,
    back_project,
    color_array,
    condition_depth,
    depth_array,
    depth_image_message,
    pack_cloud,
    rotation_matrix,
)
from cloud_timing import FrameGate, GapSampler  # noqa: E402

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
        self.cloud = pipeline(strip_nan=False).process(
            color_image(rgb), depth_image(depth), camera_info(4, 3)).cloud

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
        cloud = pipeline(strip_nan=False).process(
            color_image(rgb, encoding="bgr8"), depth_image(depth),
            camera_info(4, 3)).cloud
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
        cloud = pipeline(strip_nan=True).process(*self.frame()).cloud
        self.assertEqual((cloud.height, cloud.width), (1, 3))
        self.assertTrue(cloud.is_dense)
        self.assertEqual(cloud.row_step, 16 * 3)
        self.assertTrue(np.isfinite(decode(cloud)[["x", "y", "z"]].tolist()).all())

    def test_stripping_keeps_the_colors_of_the_finite_pixels(self):
        cloud = pipeline(strip_nan=True).process(*self.frame()).cloud
        rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        kept = [rgb[0, 0], rgb[0, 2], rgb[1, 1]]
        self.assertEqual(
            [list(colour) for colour in decode(cloud)["rgb"]],
            [[int(b), int(g), int(r), 0] for r, g, b in kept])

    def test_without_stripping_the_cloud_is_organized_and_not_dense(self):
        cloud = pipeline(strip_nan=False).process(*self.frame()).cloud
        self.assertEqual((cloud.height, cloud.width), (2, 3))
        self.assertFalse(cloud.is_dense)
        finite = np.isfinite(decode(cloud)[["x", "y", "z"]].tolist()).all(axis=1)
        self.assertEqual(finite.tolist(), [True, False, True, False, True, False])

    def test_a_frame_with_nothing_missing_is_dense_either_way(self):
        depth, rgb = sample_frame()
        for strip_nan in (True, False):
            with self.subTest(strip_nan=strip_nan):
                cloud = pipeline(strip_nan=strip_nan).process(
                    color_image(rgb), depth_image(depth), camera_info(4, 3)).cloud
                self.assertTrue(cloud.is_dense)
                self.assertEqual(cloud.width * cloud.height, 12)

    def test_a_frame_with_nothing_finite_gives_an_empty_cloud(self):
        depth = np.full((2, 3), np.nan)
        cloud = pipeline(strip_nan=True).process(
            color_image(np.zeros((2, 3, 3))), depth_image(depth), camera_info(3, 2)
        ).cloud
        self.assertEqual((cloud.height, cloud.width, len(cloud.data)), (1, 0, 0))
        self.assertTrue(cloud.is_dense)

    def test_decimation_shrinks_the_organized_grid(self):
        depth, rgb = sample_frame()
        cloud = pipeline(strip_nan=False, decimation=2).process(
            color_image(rgb), depth_image(depth), camera_info(4, 3)).cloud
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
        cloud = pipeline().process(
            color_image(rgb), depth_image(depth), camera_info(4, 3)).cloud
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
        packed = pipeline().process(
            color_image(rgb), depth_image(depth), camera_info(4, 3)).cloud
        padded = pipeline().process(
            color_image(rgb, padding=5), depth_image(depth, padding=6),
            camera_info(4, 3)).cloud
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
        return pipeline().process(color, depth, info or self.info)

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
    """The step-7 seam runs on every frame, ahead of both outputs."""

    def test_it_does_nothing_for_now(self):
        depth = np.ones((2, 2))
        self.assertIs(condition_depth(depth), depth)

    def test_it_runs_on_every_frame_but_a_cloud_is_built_only_for_delivered_ones(self):
        calls = []

        def condition(depth):
            calls.append(depth)
            return depth * 2.0

        depth, rgb = sample_frame()
        gate = FrameGate(GapSampler({3: 1.0}, 1), FRAME_PERIOD)
        target = pipeline(gate=gate, condition=condition, strip_nan=False)
        clouds, published = [], []
        for frame in range(7):
            result = target.process(
                color_image(rgb, seconds=1.0 + frame * FRAME_PERIOD),
                depth_image(depth, seconds=1.0 + frame * FRAME_PERIOD),
                camera_info(4, 3))
            published.append(result.depth)
            clouds.append(result.cloud)
        self.assertEqual(len(calls), 7)
        self.assertEqual(
            [cloud is not None for cloud in clouds],
            [True, False, False, True, False, False, True])
        # The image published and the cloud built both come from the conditioned
        # frame: twice the rendered depth.
        for frame_depth in published:
            np.testing.assert_array_equal(frame_depth, depth * 2.0)
        # Under the +90 degree z turn the depth (optical z) stays the z axis.
        np.testing.assert_allclose(
            decode(clouds[0])["z"], (depth * 2.0).ravel() + 3.0, rtol=1e-6)


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
    """The core publishes each depth frame, and a cloud only when it is delivered."""

    def setUp(self):
        self.depths, self.clouds, self.warnings = [], [], []
        gate = FrameGate(GapSampler({2: 1.0}, 1), FRAME_PERIOD)
        self.core = AdapterCore(
            pipeline(gate=gate), self.depths.append, self.clouds.append,
            self.warnings.append)
        self.depth, self.rgb = sample_frame()

    def frame(self, index, color=True, depth=True):
        seconds = 1.0 + index * FRAME_PERIOD
        if color:
            self.core.on_color(color_image(self.rgb, seconds=seconds))
        if depth:
            self.core.on_depth(depth_image(self.depth, seconds=seconds))

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

    def test_an_invalid_frame_is_dropped_whole_with_a_warning(self):
        self.core.on_camera_info(camera_info(4, 3))
        bad = depth_image(self.depth)
        bad.data = bad.data[:-4]
        self.core.on_color(color_image(self.rgb))
        self.core.on_depth(bad)
        self.assertEqual((len(self.depths), len(self.clouds)), (0, 0))
        self.assertEqual(len(self.warnings), 1)
        self.assertIn("Dropping invalid frame", self.warnings[0])
        # The next frame is fine and goes through.
        self.frame(1)
        self.assertEqual(len(self.depths), 1)

    def test_frames_wait_for_camera_info(self):
        self.frame(0)
        self.assertEqual((len(self.depths), len(self.clouds)), (0, 0))
        self.assertIn("camera_info", self.warnings[0])
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(1)
        self.assertEqual(len(self.depths), 1)

    def test_an_image_without_a_partner_is_never_published(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0, depth=False)
        self.assertEqual(len(self.depths), 0)

    def test_stragglers_are_reported(self):
        self.core.on_camera_info(camera_info(4, 3))
        self.frame(0, depth=False)
        self.frame(1)
        self.assertEqual(len(self.depths), 1)
        self.assertTrue(any("never found" in text for text in self.warnings))


if __name__ == "__main__":
    unittest.main()
