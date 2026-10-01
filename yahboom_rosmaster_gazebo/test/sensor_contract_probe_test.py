#!/usr/bin/env python3
"""Regression tests for strict evidence and CI TF sample buffering."""

from collections import deque
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


PROBE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "sensor_contract_probe.py"
)
sys.path.insert(0, str(PROBE_PATH.parent))
PROBE_SPEC = importlib.util.spec_from_file_location(
    "sensor_contract_probe", PROBE_PATH)
PROBE_MODULE = importlib.util.module_from_spec(PROBE_SPEC)
PROBE_SPEC.loader.exec_module(PROBE_MODULE)

from real_robot_contract import RealRobotContract, SOURCE_PATH  # noqa: E402

CONTRACT = RealRobotContract.load(SOURCE_PATH)


class TestSensorContractBuffering(unittest.TestCase):
    """Keep primary evidence immutable while TF samples stay recent."""

    @staticmethod
    def make_probe(performance_checks):
        """Return the capture state needed by the probe's callback."""
        return SimpleNamespace(
            first_arrivals={},
            observed_dynamic_tf_edges=set(),
            messages={"/odom": []},
            recent_tf_messages={"/odom": deque(maxlen=3)},
            required_counts={"/odom": 3},
            performance_checks=performance_checks,
        )

    @staticmethod
    def capture(probe):
        """Feed five distinguishable odometry placeholders to a probe."""
        for value in range(5):
            PROBE_MODULE.SensorContractProbe.capture(probe, "/odom", value)

    def test_primary_messages_remain_first_samples_in_both_modes(self):
        """Later valid samples must never overwrite initial contract evidence."""
        for performance_checks in (True, False):
            with self.subTest(performance_checks=performance_checks):
                probe = self.make_probe(performance_checks)
                self.capture(probe)
                self.assertEqual(probe.messages["/odom"], [0, 1, 2])

    def test_both_modes_keep_separate_recent_tf_samples(self):
        """Exact-time TF validation should use its bounded recent window."""
        for performance_checks in (True, False):
            with self.subTest(performance_checks=performance_checks):
                probe = self.make_probe(performance_checks)
                self.capture(probe)
                self.assertEqual(
                    list(probe.recent_tf_messages["/odom"]), [2, 3, 4])

    def test_the_tf_window_covers_one_odom_period_of_camera_frames(self):
        # The transform arrives once per /odom message, and tf2 does not
        # extrapolate, so the newest frames can be ahead of it; the window keeps
        # the three samples the check always had behind them.
        window = PROBE_MODULE.tf_window_size
        self.assertEqual(window(30.0, 10.0), 7)
        self.assertEqual(window(30.0, 30.0), 5)
        # A frame period is 33 ms and an odom period 102 ms: at most four
        # frames are newer than the latest transform, and three older remain.
        self.assertGreaterEqual(window(30.0, 10.0) - 4, 3)
        ledger = CONTRACT
        self.assertEqual(
            window(
                ledger.nominal("topics./cam_1/color/image_raw.rate_hz"),
                ledger.nominal("topics./odom.rate_hz")),
            7)

    def test_sample_count_is_at_least_three(self):
        """Requested counts below three must be clamped to three."""
        self.assertEqual(PROBE_MODULE.validated_sample_count(0), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(2), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(3), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(10), 10)


class TestRateGrading(unittest.TestCase):
    """The cloud's rate is its mean over the window; the others' is the median."""

    CLOUD = "/cam_1/depth/color/points"

    @staticmethod
    def probe(topic, stamps):
        """Return the state validate_rate reads, with messages stamped ``stamps``."""
        messages = [
            SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))))
            for stamp in stamps
        ]
        return SimpleNamespace(
            messages={topic: messages},
            stamp_seconds=PROBE_MODULE.SensorContractProbe.stamp_seconds)

    def grade(self, topic, stamps, minimum, maximum):
        errors = []
        PROBE_MODULE.SensorContractProbe.validate_rate(
            self.probe(topic, stamps), topic, minimum, maximum, errors)
        return errors

    def test_a_window_of_uneven_gaps_is_graded_by_its_mean_rate(self):
        # Eight one-frame gaps and one ten-frame gap: the median period is one
        # frame, 30 Hz, but 18 frames pass in nine clouds, a mean of 15.2 Hz.
        frames = [0, 1, 2, 3, 4, 5, 6, 7, 8, 18]
        stamps = [10.0 + 0.033 * frame for frame in frames]
        self.assertEqual(self.grade(self.CLOUD, stamps, 14.0, 16.0), [])
        self.assertTrue(self.grade(self.CLOUD, stamps, 28.0, 33.0))

    def test_a_long_gap_pulls_the_cloud_rate_down(self):
        frames = [0, 1, 2, 3, 4, 5, 6, 7, 8, 40]
        stamps = [10.0 + 0.033 * frame for frame in frames]
        self.assertTrue(self.grade(self.CLOUD, stamps, 15.0, 33.0))

    def test_other_topics_keep_the_median_period(self):
        # One long gap among steady 33 ms periods leaves the median at 30 Hz.
        frames = [0, 1, 2, 3, 4, 5, 6, 7, 8, 40]
        stamps = [10.0 + 0.033 * frame for frame in frames]
        self.assertEqual(self.grade("/cam_1/color/image_raw", stamps, 27.0, 33.0), [])

    def test_the_cloud_is_the_only_mean_graded_topic(self):
        self.assertEqual(PROBE_MODULE.MEAN_RATE_TOPICS, (self.CLOUD,))


class TestDepthQualityInputs(unittest.TestCase):
    """The probe reads the adapter's own input and decodes it with the row stride."""

    def test_the_raw_topic_is_the_one_the_adapter_subscribes_to(self):
        adapter = (PROBE_PATH.parent / "camera_adapter.py").read_text(encoding="utf-8")
        self.assertIn(f'"{PROBE_MODULE.RAW_DEPTH_TOPIC}"', adapter)
        self.assertIn(f'"{PROBE_MODULE.PUBLIC_DEPTH_TOPIC}"', adapter)

    def test_a_padded_depth_image_decodes_to_its_pixels(self):
        pixels = np.arange(6, dtype="<f4").reshape(2, 3)
        rows = np.zeros((2, 4), dtype="<f4")
        rows[:, :3] = pixels
        message = SimpleNamespace(
            encoding="32FC1", height=2, width=3, step=16, data=rows.tobytes())
        decoded = PROBE_MODULE.SensorContractProbe.depth_pixels(message)
        np.testing.assert_array_equal(decoded, pixels)
        self.assertEqual(decoded.dtype, np.float32)

    def test_another_encoding_is_refused(self):
        message = SimpleNamespace(encoding="16UC1", height=1, width=1, step=2, data=b"")
        with self.assertRaises(ValueError):
            PROBE_MODULE.SensorContractProbe.depth_pixels(message)

    def test_the_probe_grades_the_physical_profile_by_default(self):
        launch = (PROBE_PATH.parents[1] / "test" / "sensor_contract.launch.py").read_text(
            encoding="utf-8")
        self.assertIn(
            'DeclareLaunchArgument(\n            "sensor_profile", default_value="physical"',
            launch)


class TestSensorContractLedger(unittest.TestCase):
    """The probe grades rates from the ledger and pins what it hard-codes."""

    def test_every_graded_topic_has_a_rate_contract(self):
        for topic in PROBE_MODULE.RATE_GRADED_TOPICS:
            with self.subTest(topic=topic):
                minimum, maximum = CONTRACT.rate_bounds(topic)
                self.assertLess(minimum, maximum)

    def test_best_effort_topics_match_the_ledger(self):
        best_effort = {
            key.split(".")[1]
            for key, entry in CONTRACT.entries()
            if key.endswith(".reliability")
            and entry["nominal"] == "best_effort"
        }
        self.assertEqual(set(PROBE_MODULE.BEST_EFFORT_TOPICS), best_effort)

    def test_wheel_joints_match_the_ledger(self):
        self.assertEqual(
            PROBE_MODULE.EXPECTED_WHEEL_JOINTS,
            set(CONTRACT.nominal("joint_states.names")),
        )


if __name__ == "__main__":
    unittest.main()
