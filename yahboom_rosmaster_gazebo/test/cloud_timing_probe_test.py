#!/usr/bin/env python3
"""Unit tests for how the cloud timing probe grades stamps, with no ROS graph."""

from pathlib import Path
import random
import sys
import unittest

import numpy as np
from builtin_interfaces.msg import Time

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from camera_adapter import pack_cloud  # noqa: E402
from cloud_timing import GapSampler  # noqa: E402
from cloud_timing_probe import grade_timing, layout_errors  # noqa: E402
from sensor_profiles import SOURCE_PATH, load_sensor_profile  # noqa: E402

PHYSICAL = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")
IDEAL = load_sensor_profile(SOURCE_PATH, "point_cloud", "ideal")
FRAME_NS = 33_000_000
START_NS = 1_000_000_000


def image_stamps(frames):
    return [START_NS + FRAME_NS * frame for frame in range(frames)]


def physical_run(seed=1, gaps=500, latency=0.050, jitter=0.001):
    """Return the stamps, latencies and images of a correct physical profile run."""
    sampler = GapSampler(PHYSICAL["gap_frames"], seed)
    frame, stamps = 0, [START_NS]
    for _ in range(gaps):
        frame += sampler.next_gap()
        stamps.append(START_NS + FRAME_NS * frame)
    noise = random.Random(seed)
    latencies = [latency + noise.uniform(0.0, jitter) for _ in stamps]
    return stamps, latencies, image_stamps(frame + 1)


def grade(stamps, latencies, images, profile_name="physical", depth_latencies=(0.020,) * 5,
          frames=None):
    """Grade a run, in which the camera's frames are ``images`` unless ``frames`` says so."""
    profile = PHYSICAL if profile_name == "physical" else IDEAL
    return grade_timing(
        stamps, latencies, images, images if frames is None else frames,
        list(depth_latencies), profile_name, profile)


class TestPhysicalGrading(unittest.TestCase):
    """A run drawn from the measured table passes, and anything else fails."""

    def test_a_correct_run_passes_over_many_seeds(self):
        for seed in range(40):
            with self.subTest(seed=seed):
                errors, _ = grade(*physical_run(seed))
                self.assertEqual(errors, [])

    def test_the_statistics_describe_the_run(self):
        _, stats = grade(*physical_run(3))
        self.assertEqual(stats["frame_period_ms"], 33.0)
        self.assertEqual(stats["gaps"], 500)
        self.assertEqual(stats["off_grid_gaps"], 0)
        self.assertAlmostEqual(stats["median_latency_ms"], 50.5, delta=0.5)
        self.assertAlmostEqual(stats["mean_gap_frames"], 3.63, delta=0.5)

    def test_a_stamp_off_the_frame_grid_fails(self):
        stamps, latencies, images = physical_run()
        stamps[10] += 15_000_000
        errors, stats = grade(stamps, latencies, images)
        self.assertGreaterEqual(stats["off_grid_gaps"], 1)
        self.assertTrue(any("whole number" in error for error in errors), errors)

    def test_two_milliseconds_of_stamp_wander_is_still_on_grid(self):
        # The simulated camera's frames come 32, 33 or 34 ms apart, so a
        # stamp wanders up to 2 ms from a 33 ms grid, and a gap of many
        # frames can be 2 ms long or short.
        stamps, latencies, images = physical_run()
        walk = random.Random(9)
        offsets, offset = [], 1
        for _ in range(len(images)):
            offset = min(2, max(0, offset + walk.choice((-1, 0, 0, 1))))
            offsets.append(offset * 1_000_000)

        def wander(stamp):
            return stamp + offsets[(stamp - START_NS) // FRAME_NS]

        errors, stats = grade(
            [wander(stamp) for stamp in stamps], latencies,
            [wander(stamp) for stamp in images])
        self.assertEqual(stats["off_grid_gaps"], 0, errors)
        self.assertEqual(errors, [])

    def test_four_milliseconds_off_the_grid_fails(self):
        stamps, latencies, images = physical_run()
        stamps[10] += 4_000_000
        errors, stats = grade(stamps, latencies, images)
        self.assertGreaterEqual(stats["off_grid_gaps"], 1)

    def test_clouds_stamped_off_any_frame_fail(self):
        # Stamping a cloud with the time it was published, not its frame's.
        stamps, latencies, images = physical_run()
        stamps = [stamp + 7_000_000 for stamp in stamps]
        errors, stats = grade(stamps, latencies, images)
        self.assertEqual(stats["stamps_matching_no_frame"], len(stamps))
        self.assertTrue(any("match no camera frame" in error for error in errors), errors)

    def test_a_cloud_may_match_a_frame_only_the_depth_stream_saw(self):
        stamps, latencies, images = physical_run()
        color_only = [stamp for stamp in images if stamp not in set(stamps[:20])]
        errors, stats = grade(stamps, latencies, color_only, frames=images)
        self.assertEqual(stats["stamps_matching_no_frame"], 0, errors)

    def test_every_frame_delivered_is_not_the_physical_distribution(self):
        frames = 600
        stamps = image_stamps(frames)
        errors, _ = grade(stamps, [0.05] * frames, image_stamps(frames))
        self.assertTrue(any("median gap" in error for error in errors), errors)

    def test_an_overlong_gap_pattern_fails(self):
        stamps = image_stamps(4000)[::12]
        errors, _ = grade(stamps, [0.05] * len(stamps), image_stamps(4000))
        self.assertTrue(any("median gap" in error for error in errors), errors)

    def test_the_wrong_latency_fails(self):
        for latency in (0.020, 0.070):
            with self.subTest(latency=latency):
                errors, _ = grade(*physical_run(latency=latency))
                self.assertTrue(any("median latency" in error for error in errors), errors)

    def test_a_stall_longer_than_the_table_fails(self):
        stamps, latencies, images = physical_run()
        stamps = stamps[:100] + [stamp + 60 * FRAME_NS for stamp in stamps[100:]]
        images = image_stamps((stamps[-1] - START_NS) // FRAME_NS + 1)
        errors, _ = grade(stamps, latencies, images)
        self.assertTrue(any("longest gap" in error for error in errors), errors)

    def test_too_few_gaps_to_grade_a_distribution_fail(self):
        errors, _ = grade(*physical_run(gaps=10))
        self.assertTrue(any("fewer than" in error for error in errors), errors)

    def test_missing_messages_fail(self):
        self.assertTrue(grade([START_NS], [0.05], image_stamps(2))[0])

    def test_non_increasing_stamps_fail(self):
        stamps, latencies, images = physical_run()
        stamps[5] = stamps[4]
        errors, _ = grade(stamps, latencies, images)
        self.assertIn("cloud stamps do not strictly increase", errors)


class TestStartupBurst(unittest.TestCase):
    """Gazebo's camera catches up on owed frames when it starts; those are not graded."""

    def test_clouds_made_from_a_startup_burst_are_skipped(self):
        stamps, latencies, images = physical_run(seed=5)
        # 40 frames a millisecond apart before the run, and 14 clouds made
        # from them, a few milliseconds apart.
        burst = [START_NS - 80_000_000 + 1_000_000 * step for step in range(40)]
        burst_clouds = burst[::3]
        errors, stats = grade(
            burst_clouds + stamps, [0.05] * len(burst_clouds) + latencies,
            burst + images)
        self.assertEqual(errors, [])
        self.assertEqual(stats["startup_burst_clouds_skipped"], len(burst_clouds))
        self.assertEqual(stats["gaps"], 500)

    def test_the_burst_clouds_would_fail_without_the_cut(self):
        stamps, latencies, images = physical_run(seed=5)
        burst = [START_NS - 80_000_000 + 1_000_000 * step for step in range(40)]
        errors, stats = grade(
            burst[::3] + stamps, [0.05] * 14 + latencies, images)
        self.assertTrue(errors)
        self.assertEqual(stats["startup_burst_clouds_skipped"], 0)

    def test_a_run_with_no_burst_skips_nothing(self):
        _, stats = grade(*physical_run())
        self.assertEqual(stats["startup_burst_clouds_skipped"], 0)


class TestIdealGrading(unittest.TestCase):
    """The ideal profile delivers every frame with no latency of its own."""

    def run_of(self, frames=200, latency=0.021, depth=0.020):
        stamps = image_stamps(frames)
        return stamps, [latency] * frames, image_stamps(frames), (depth,) * frames

    def test_every_frame_at_the_depth_image_latency_passes(self):
        stamps, latencies, images, depths = self.run_of()
        errors, stats = grade(stamps, latencies, images, "ideal", depths)
        self.assertEqual(errors, [])
        self.assertEqual(stats["gaps_over_one_frame"], 0)

    def test_a_lost_cloud_or_two_is_the_transport_not_the_profile(self):
        stamps, latencies, images, depths = self.run_of()
        for index in (150, 100, 50):
            del stamps[index], latencies[index]
        errors, stats = grade(stamps, latencies, images, "ideal", depths)
        self.assertEqual(errors, [])
        self.assertEqual(stats["gaps_over_one_frame"], 3)

    def test_every_fourth_frame_skipped_fails(self):
        stamps, latencies, images, depths = self.run_of()
        stamps = [stamp for index, stamp in enumerate(stamps) if index % 4]
        errors, _ = grade(stamps, [0.021] * len(stamps), images, "ideal", depths)
        self.assertTrue(
            any("only" in error and "every frame" in error for error in errors), errors)

    def test_a_long_stall_fails(self):
        stamps, latencies, images, depths = self.run_of(frames=400)
        stamps = stamps[:100] + stamps[110:]
        errors, _ = grade(stamps, [0.021] * len(stamps), images, "ideal", depths)
        self.assertTrue(any("a gap of" in error for error in errors), errors)

    def test_added_latency_fails(self):
        stamps, latencies, images, depths = self.run_of(latency=0.070)
        errors, _ = grade(stamps, latencies, images, "ideal", depths)
        self.assertTrue(any("adds no latency" in error for error in errors), errors)


class TestLayout(unittest.TestCase):
    """The probe grades the layout the adapter packs."""

    @staticmethod
    def cloud(strip_nan=True):
        points = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0], [4.0, 5.0, 6.0]])
        return pack_cloud(
            points, np.zeros(3, dtype=np.uint32), np.eye(3), (0.0, 0.0, 0.0),
            "cam_1_depth_frame", Time(sec=1), 1, 3, strip_nan)

    def test_the_adapters_default_cloud_has_no_layout_errors(self):
        self.assertEqual(layout_errors(self.cloud()), [])

    def test_an_organized_or_padded_cloud_is_reported(self):
        cloud = self.cloud(strip_nan=False)
        self.assertTrue(any("is_dense" in error for error in layout_errors(cloud)))
        cloud = self.cloud()
        cloud.point_step = 24
        self.assertTrue(any("point_step" in error for error in layout_errors(cloud)))
        cloud = self.cloud()
        cloud.header.frame_id = "cam_1_color_frame"
        self.assertTrue(any("frame" in error for error in layout_errors(cloud)))
        cloud = self.cloud()
        cloud.height, cloud.width = 2, 1
        self.assertTrue(any("height" in error for error in layout_errors(cloud)))


if __name__ == "__main__":
    unittest.main()
