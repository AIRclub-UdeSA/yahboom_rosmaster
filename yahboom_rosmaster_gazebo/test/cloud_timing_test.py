#!/usr/bin/env python3
"""Unit tests for the point cloud's timing model, with no ROS graph."""

from collections import Counter
import math
from pathlib import Path
import sys
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from cloud_timing import (  # noqa: E402
    FrameGate,
    GapSampler,
    cumulative_within,
    gap_statistics,
    normalized,
    quantile,
    quantile_interval,
    publish_delay,
    resolve_seed,
)
from sensor_profiles import SOURCE_PATH, load_sensor_profile  # noqa: E402

FRAME_PERIOD = 0.033
PHYSICAL = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")
IDEAL = load_sensor_profile(SOURCE_PATH, "point_cloud", "ideal")
TABLE = normalized(PHYSICAL["gap_frames"])
DRAWS = 100_000


def gate_over(gaps, frames, seed=1):
    """Return the frame numbers a gate delivers among ``frames`` camera frames."""
    gate = FrameGate(GapSampler(gaps, seed), FRAME_PERIOD)
    return [
        frame for frame in range(frames)
        if gate.deliver(1.0 + frame * FRAME_PERIOD)
    ]


class TestGapSampler(unittest.TestCase):
    """The sampler reproduces the measured table, and only from its seed."""

    def test_the_same_seed_gives_the_same_sequence(self):
        first = GapSampler(PHYSICAL["gap_frames"], 7)
        second = GapSampler(PHYSICAL["gap_frames"], 7)
        self.assertEqual(
            [first.next_gap() for _ in range(1000)],
            [second.next_gap() for _ in range(1000)])

    def test_different_seeds_give_different_sequences(self):
        first = GapSampler(PHYSICAL["gap_frames"], 7)
        second = GapSampler(PHYSICAL["gap_frames"], 8)
        self.assertNotEqual(
            [first.next_gap() for _ in range(1000)],
            [second.next_gap() for _ in range(1000)])

    def test_the_empirical_distribution_matches_the_table(self):
        """
        Bound the largest error at 0.008, about five standard deviations.

        The most probable gap, one frame at 0.331, has the largest standard
        deviation over 100000 draws: sqrt(0.331 * 0.669 / 100000) = 0.0015.
        A fixed seed makes the check deterministic; the bound says how far a
        correct sampler could stray for any seed.
        """
        sampler = GapSampler(PHYSICAL["gap_frames"], 20260926)
        counts = Counter(sampler.next_gap() for _ in range(DRAWS))
        self.assertLessEqual(set(counts), set(TABLE))
        worst = max(
            abs(counts[gap] / DRAWS - probability)
            for gap, probability in TABLE.items())
        self.assertLess(worst, 0.008)

    def test_every_gap_in_the_table_can_be_drawn(self):
        sampler = GapSampler({1: 0.5, 5: 0.5}, 3)
        self.assertEqual({sampler.next_gap() for _ in range(200)}, {1, 5})

    def test_the_ideal_profile_draws_no_gaps(self):
        sampler = GapSampler(IDEAL["gap_frames"], 3)
        self.assertEqual({sampler.next_gap() for _ in range(1000)}, {1})

    def test_a_malformed_distribution_is_rejected(self):
        for distribution in ({}, {0: 1.0}, {1.5: 1.0}, {1: 0.0}, {1: -1.0}, {1: float("nan")}):
            with self.subTest(distribution=distribution):
                with self.assertRaises(ValueError):
                    GapSampler(distribution, 1)


class TestSeed(unittest.TestCase):
    """A negative seed means a random one, which the launch logs."""

    def test_a_given_seed_is_kept(self):
        self.assertEqual(resolve_seed(42), 42)
        self.assertEqual(resolve_seed(0), 0)

    def test_a_negative_seed_is_replaced_by_a_random_one(self):
        seeds = {resolve_seed(-1) for _ in range(20)}
        self.assertGreater(len(seeds), 1)
        self.assertTrue(all(seed >= 0 for seed in seeds))


class TestGapStatistics(unittest.TestCase):
    """The table's summary is the physical measurement's."""

    def test_the_table_reproduces_the_published_summary(self):
        statistics = gap_statistics(PHYSICAL["gap_frames"])
        self.assertEqual(statistics["median"], 2)
        self.assertEqual(statistics["p95"], 11)
        self.assertEqual(statistics["longest"], 34)
        # The table carries four decimals, so its mean is 3.625 against the 3.63
        # of the 988 counted gaps.
        self.assertAlmostEqual(statistics["mean"], 3.63, delta=0.01)

    def test_a_single_gap_is_every_statistic(self):
        self.assertEqual(
            gap_statistics({1: 1.0}),
            {"mean": 1.0, "median": 1, "p95": 1, "longest": 1})


class TestQuantileInterval(unittest.TestCase):
    """The tolerance a test derives from its sample size holds for a correct sampler."""

    def test_quantiles_read_the_table_back(self):
        self.assertEqual(quantile(PHYSICAL["gap_frames"], 0.5), 2)
        self.assertEqual(quantile(PHYSICAL["gap_frames"], 0.95), 11)
        self.assertEqual(quantile(PHYSICAL["gap_frames"], 0.0), 1)
        self.assertEqual(quantile(PHYSICAL["gap_frames"], 1.5), 34)

    def test_the_cumulative_probability_sums_the_table(self):
        self.assertAlmostEqual(cumulative_within(PHYSICAL["gap_frames"], 2), 0.5163, places=3)
        self.assertAlmostEqual(cumulative_within(PHYSICAL["gap_frames"], 34), 1.0)

    def test_the_interval_narrows_as_the_sample_grows(self):
        widths = []
        for samples in (100, 500, 5000):
            low, high = quantile_interval(PHYSICAL["gap_frames"], 0.95, samples)
            widths.append(high - low)
        self.assertEqual(widths, sorted(widths, reverse=True))

    def test_the_median_and_p95_intervals_at_the_launch_test_sample_size(self):
        # 60 s of sim time at 30.3 frames a second and 3.63 frames a gap.
        samples = 500
        self.assertEqual(quantile_interval(PHYSICAL["gap_frames"], 0.5, samples), (2, 3))
        low, high = quantile_interval(PHYSICAL["gap_frames"], 0.95, samples)
        self.assertLess(low, 11)
        self.assertGreater(high, 11)

    def test_a_correct_sampler_stays_inside_over_many_seeds(self):
        samples, trials = 500, 400
        misses = 0
        for seed in range(trials):
            sampler = GapSampler(PHYSICAL["gap_frames"], seed)
            draws = sorted(sampler.next_gap() for _ in range(samples))
            for fraction in (0.5, 0.95):
                observed = draws[max(0, math.ceil(fraction * samples) - 1)]
                low, high = quantile_interval(PHYSICAL["gap_frames"], fraction, samples)
                misses += not low <= observed <= high
        # 4 standard deviations is a miss probability of about 6e-5 a quantile.
        self.assertEqual(misses, 0)


class TestFrameGate(unittest.TestCase):
    """The gate delivers a drawn number of frames after the last delivery."""

    def test_the_first_frame_is_delivered(self):
        self.assertEqual(gate_over({4: 1.0}, 1), [0])

    def test_frames_are_delivered_a_gap_apart(self):
        self.assertEqual(gate_over({4: 1.0}, 13), [0, 4, 8, 12])

    def test_ideal_delivers_every_frame(self):
        self.assertEqual(gate_over(IDEAL["gap_frames"], 50), list(range(50)))

    def test_physical_gaps_are_the_drawn_ones(self):
        sampler = GapSampler(PHYSICAL["gap_frames"], 5)
        frames = 1000
        expected = [sampler.next_gap() for _ in range(frames)]
        delivered = gate_over(PHYSICAL["gap_frames"], frames, seed=5)
        gaps = [later - earlier for earlier, later in zip(delivered, delivered[1:])]
        # The first draw follows the first frame, so the gaps are the sequence.
        self.assertEqual(gaps, expected[:len(gaps)])

    def test_a_missing_frame_delays_the_delivery_to_the_next_one(self):
        gate = FrameGate(GapSampler({3: 1.0}, 1), FRAME_PERIOD)
        stamps = [1.0 + frame * FRAME_PERIOD for frame in (0, 1, 2, 4, 5, 6, 7)]
        delivered = [gate.deliver(stamp) for stamp in stamps]
        # Frame 3 is missing, so frame 4 is the first at or after the target.
        self.assertEqual(delivered, [True, False, False, True, False, False, True])

    def test_a_millisecond_of_stamp_jitter_does_not_shift_the_count(self):
        gate = FrameGate(GapSampler({20: 1.0}, 1), FRAME_PERIOD)
        stamp, delivered = 1.0, []
        for frame in range(41):
            # Stamps alternate between 33 and 34 ms apart.
            stamp += 0.034 if frame % 2 else 0.033
            delivered.append(gate.deliver(stamp))
        self.assertEqual([i for i, hit in enumerate(delivered) if hit], [0, 20, 40])

    def test_a_repeated_or_reordered_stamp_is_not_a_new_frame(self):
        gate = FrameGate(GapSampler({2: 1.0}, 1), FRAME_PERIOD)
        self.assertTrue(gate.deliver(1.0))
        self.assertFalse(gate.deliver(1.0))
        self.assertFalse(gate.deliver(0.9))
        self.assertFalse(gate.deliver(1.0 + FRAME_PERIOD))
        self.assertTrue(gate.deliver(1.0 + 2 * FRAME_PERIOD))

    def test_a_startup_burst_of_frames_makes_a_few_clouds_not_one_each(self):
        # Gazebo's camera catches up on the frames it owes when it starts, with
        # stamps a millisecond or so apart. Delivering every frame, ideal
        # publishes a cloud once per half frame period of that burst at most,
        # not one per burst frame.
        gate = FrameGate(GapSampler({1: 1.0}, 1), FRAME_PERIOD)
        stamps = [1.0 + 0.001 * step for step in range(100)]
        stamps += [1.099 + FRAME_PERIOD * frame for frame in range(1, 6)]
        delivered = [stamp for stamp in stamps if gate.deliver(stamp)]
        burst = [stamp for stamp in delivered if stamp < 1.1]
        self.assertLessEqual(len(burst), 7)
        self.assertGreaterEqual(len(burst), 3)
        # Once the frames settle to the frame period, each is a cloud again.
        self.assertEqual(len(delivered) - len(burst), 5)
        gaps = [later - earlier for earlier, later in zip(delivered, delivered[1:])]
        self.assertGreater(min(gaps), FRAME_PERIOD / 2 - 0.001)

    def test_a_frame_just_under_half_a_period_later_is_not_counted(self):
        gate = FrameGate(GapSampler({1: 1.0}, 1), FRAME_PERIOD)
        self.assertTrue(gate.deliver(1.0))
        self.assertFalse(gate.deliver(1.0 + 0.4 * FRAME_PERIOD))
        # The next is measured from the last frame counted, not the burst frame.
        self.assertTrue(gate.deliver(1.0 + 0.9 * FRAME_PERIOD))

    def test_a_bad_frame_period_is_rejected(self):
        for period in (0.0, -0.033, float("nan")):
            with self.subTest(period=period):
                with self.assertRaises(ValueError):
                    FrameGate(GapSampler({1: 1.0}, 1), period)


class TestPublishDelay(unittest.TestCase):
    """A cloud leaves at its capture stamp plus the latency, or at once if late."""

    def test_an_early_cloud_waits_for_stamp_plus_latency(self):
        delay, late = publish_delay(stamp_s=10.0, now_s=10.020, latency_s=0.050)
        self.assertAlmostEqual(delay, 0.030)
        self.assertFalse(late)

    def test_the_wait_is_measured_from_the_stamp_not_from_arrival(self):
        # The images arrive 20 ms after the stamp; the cloud still leaves at
        # 50 ms after it, not at 70.
        delay, _ = publish_delay(stamp_s=10.0, now_s=10.020, latency_s=0.050)
        self.assertAlmostEqual(10.020 + delay, 10.050)

    def test_a_late_cloud_goes_out_at_once_and_is_counted(self):
        delay, late = publish_delay(stamp_s=10.0, now_s=10.070, latency_s=0.050)
        self.assertEqual(delay, 0.0)
        self.assertTrue(late)

    def test_a_cloud_ready_exactly_on_time_is_not_late(self):
        delay, late = publish_delay(stamp_s=10.0, now_s=10.050, latency_s=0.050)
        self.assertAlmostEqual(delay, 0.0)
        self.assertFalse(late)

    def test_ideal_adds_no_delay_and_nothing_is_late(self):
        delay, late = publish_delay(
            stamp_s=10.0, now_s=10.020, latency_s=IDEAL["latency_s"])
        self.assertEqual(delay, 0.0)
        self.assertFalse(late)


if __name__ == "__main__":
    unittest.main()
