#!/usr/bin/env python3
"""Keep the depth quality grader honest: it must pass the physical camera and fail others."""

from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from depth_quality import (  # noqa: E402
    FramePairs,
    depth_quality_errors,
    expectations_from_ledger,
)
from real_robot_contract import RealRobotContract, SOURCE_PATH  # noqa: E402

CONTRACT = RealRobotContract.load(SOURCE_PATH)
EXPECTED = expectations_from_ledger(CONTRACT)
SHAPE = (240, 320)
CAMERA_HEIGHT_TIMES_FY = 29.6  # about 0.109 m * 271.8 px: the floor row r sits at 29.6 / r


def floor_scene():
    """Return a frame like empty.world's: floor from 0.25 m to the far clip, then none."""
    depth = np.full(SHAPE, np.inf, dtype=np.float32)
    rows_below = np.arange(1, SHAPE[0] - 120 + 1)
    floor = CAMERA_HEIGHT_TIMES_FY / (rows_below - 0.5)
    floor[floor > 8.0] = np.inf
    depth[120:, :] = floor[:, None].astype(np.float32)
    return depth


def camera(rendered, seed, scale=None, floor=0.002, coefficient=0.0019,
           exponent=2.36, minimum=0.6, cut=True):
    """
    Return a frame from an independently written model of the physical camera.

    Float64 and numpy's default generator, apart from the adapter entirely.
    ``scale`` defaults to the inverse of the calibration doc's 1.012.
    """
    scale = 1.0 / 1.012 - 1.0 if scale is None else scale
    reported = rendered.astype(np.float64) * (1.0 + scale)
    generator = np.random.default_rng(seed)
    with np.errstate(invalid="ignore", over="ignore"):
        sigma = np.maximum(floor, coefficient * reported ** exponent)
        published = reported + sigma * generator.standard_normal(rendered.shape)
        if cut:
            published[~(published >= minimum)] = np.nan
    published[~np.isfinite(rendered)] = np.nan
    return published.astype(np.float32)


def pairs(**model):
    rendered = floor_scene()
    return [(rendered, camera(rendered, seed, **model)) for seed in (1, 2, 3)]


class TestPhysicalGrading(unittest.TestCase):
    """The grader accepts the model of the physical camera and names each departure."""

    def errors(self, given):
        return depth_quality_errors(given, "physical", EXPECTED)

    def test_the_expectations_come_from_the_physical_ledger(self):
        self.assertEqual(EXPECTED["scale_min"], -0.013)
        self.assertEqual(EXPECTED["scale_max"], -0.011)
        self.assertEqual(EXPECTED["min_range_m"], 0.6)
        self.assertEqual(EXPECTED["sigma_floor_m"], 0.002)
        self.assertEqual(EXPECTED["sigma_coefficient"], 0.0019)
        self.assertEqual(EXPECTED["sigma_exponent"], 2.36)

    def test_the_model_of_the_physical_camera_passes(self):
        for seed_offset in range(5):
            rendered = floor_scene()
            given = [
                (rendered, camera(rendered, 100 * seed_offset + seed)) for seed in (1, 2, 3)]
            with self.subTest(seed_offset=seed_offset):
                self.assertEqual(self.errors(given), [])

    def test_a_scale_error_of_zero_fails(self):
        self.assertTrue(any("scale error" in e for e in self.errors(pairs(scale=0.0))))

    def test_a_scale_error_outside_the_range_fails_either_side(self):
        for scale in (-0.02, -0.005):
            with self.subTest(scale=scale):
                self.assertTrue(
                    any("scale error" in e for e in self.errors(pairs(scale=scale))))

    def test_a_wrong_noise_exponent_fails(self):
        errors = self.errors(pairs(exponent=2.0))
        self.assertTrue(any("noise at" in e for e in errors), errors)

    def test_no_noise_or_double_noise_fails(self):
        for model in ({"floor": 0.0, "coefficient": 0.0}, {"floor": 0.004}):
            with self.subTest(model=model):
                errors = self.errors(pairs(**model))
                self.assertTrue(any("noise at" in e for e in errors), errors)

    def test_a_missing_cutoff_fails(self):
        errors = self.errors(pairs(cut=False))
        self.assertTrue(any("below the 0.6 m" in e for e in errors), errors)

    def test_a_cutoff_too_high_fails(self):
        errors = self.errors(pairs(minimum=1.0))
        self.assertTrue(any("over the 0.6 m minimum range are NaN" in e for e in errors), errors)

    def test_one_published_pixel_under_the_minimum_fails(self):
        given = pairs()
        given[1][1][239, 7] = 0.599
        self.assertTrue(any("below the 0.6 m" in e for e in self.errors(given)))

    def test_one_infinite_pixel_fails(self):
        given = pairs()
        given[0][1][0, 0] = np.inf
        self.assertTrue(any("infinite" in e for e in self.errors(given)))

    def test_a_finite_pixel_where_nothing_was_rendered_fails(self):
        given = pairs()
        given[0][1][0, 0] = 3.0
        self.assertTrue(any("no render return are finite" in e for e in self.errors(given)))

    def test_a_scene_with_no_spread_of_distances_cannot_be_graded(self):
        rendered = np.full(SHAPE, 2.0, dtype=np.float32)
        given = [(rendered, camera(rendered, seed)) for seed in (1, 2, 3)]
        self.assertTrue(any("distance bands" in e for e in self.errors(given)))

    def test_no_frames_is_an_error(self):
        self.assertTrue(depth_quality_errors([], "physical", EXPECTED))

    def test_a_shape_mismatch_is_named(self):
        errors = self.errors([(floor_scene(), np.zeros((2, 2), np.float32))])
        self.assertTrue(any("published" in e for e in errors))


class TestIdealGrading(unittest.TestCase):
    """Ideal publishes every finite pixel bit for bit."""

    def errors(self, given):
        return depth_quality_errors(given, "ideal", EXPECTED)

    def test_the_render_with_nan_for_no_return_passes(self):
        rendered = floor_scene()
        published = np.where(np.isfinite(rendered), rendered, np.float32(np.nan))
        self.assertEqual(self.errors([(rendered, published)]), [])

    def test_one_changed_bit_fails(self):
        rendered = floor_scene()
        published = np.where(np.isfinite(rendered), rendered, np.float32(np.nan))
        published[200, 5] = np.nextafter(published[200, 5], np.float32(10.0))
        self.assertTrue(any("bit for bit" in e for e in self.errors([(rendered, published)])))

    def test_infinity_left_in_place_fails(self):
        rendered = floor_scene()
        self.assertTrue(any("infinite" in e for e in self.errors([(rendered, rendered)])))

    def test_the_physical_camera_is_not_ideal(self):
        self.assertTrue(self.errors(pairs()))


class TestFramePairs(unittest.TestCase):
    """Rendered and published frames are matched by exact stamp, in either order."""

    def test_a_pair_completes_in_either_order(self):
        frames = FramePairs()
        frames.add("rendered", (1, 0), "r1")
        frames.add("published", (1, 0), "p1")
        frames.add("published", (2, 0), "p2")
        frames.add("rendered", (2, 0), "r2")
        self.assertEqual(frames.pairs, [("r1", "p1"), ("r2", "p2")])

    def test_stamps_that_differ_at_all_never_pair(self):
        frames = FramePairs()
        frames.add("rendered", (1, 0), "r")
        frames.add("published", (1, 1), "p")
        self.assertEqual(frames.pairs, [])

    def test_a_frame_that_never_pairs_is_evicted_and_holding_stays_small(self):
        frames = FramePairs(hold=3)
        for index in range(10):
            frames.add("rendered", (index, 0), index)
        frames.add("published", (0, 0), "late")
        self.assertEqual(frames.pairs, [])
        frames.add("published", (9, 0), "recent")
        self.assertEqual(frames.pairs, [(9, "recent")])


if __name__ == "__main__":
    unittest.main()
