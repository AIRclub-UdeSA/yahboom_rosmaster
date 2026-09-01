#!/usr/bin/env python3
"""Regression tests for strict evidence and CI TF sample buffering."""

from collections import deque
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


PROBE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "sensor_contract_probe.py"
)
PROBE_SPEC = importlib.util.spec_from_file_location(
    "sensor_contract_probe", PROBE_PATH)
PROBE_MODULE = importlib.util.module_from_spec(PROBE_SPEC)
PROBE_SPEC.loader.exec_module(PROBE_MODULE)


class TestSensorContractBuffering(unittest.TestCase):
    """Keep primary evidence immutable while CI TF samples stay recent."""

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

    def test_correctness_mode_keeps_separate_recent_tf_samples(self):
        """CI exact-time TF validation should use its bounded recent window."""
        probe = self.make_probe(False)
        self.capture(probe)
        self.assertEqual(list(probe.recent_tf_messages["/odom"]), [2, 3, 4])

    def test_sample_count_supports_third_from_last_lookup(self):
        """Requested counts below three must be clamped to three."""
        self.assertEqual(PROBE_MODULE.validated_sample_count(0), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(2), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(3), 3)
        self.assertEqual(PROBE_MODULE.validated_sample_count(10), 10)


if __name__ == "__main__":
    unittest.main()
