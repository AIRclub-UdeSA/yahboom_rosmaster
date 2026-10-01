#!/usr/bin/env python3
"""Keep the measurement tools under tools/ honest without running a simulator."""

import os
from pathlib import Path
import re
import sys
import tempfile
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "tools"))
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

import measure_cloud_timing  # noqa: E402
import sim_run  # noqa: E402

PHYSICAL_REPOSITORY = Path("~/Documents/air-club/physical_rosmaster").expanduser()


def probe_result(gaps, latencies, depth_valid):
    """Return a probe result shaped like sensor_capability_probe.py's JSON."""
    stamps = [10.0]
    for gap in gaps:
        stamps.append(stamps[-1] + gap / measure_cloud_timing.FRAME_RATE_HZ)
    histogram = {}
    for gap in gaps:
        histogram[str(gap)] = histogram.get(str(gap), 0) + 1
    depth_stamps = [10.0 + index / measure_cloud_timing.FRAME_RATE_HZ for index in range(31)]
    depth = {"valid_fraction": depth_valid, "nan_fraction": 1.0 - depth_valid}
    return {
        "measurement": {"probe_git_blob": "abc"},
        "topics": [
            {
                "topic": measure_cloud_timing.CLOUD, "rate_hz": 8.0,
                "frame_cadence": {"frames_per_gap": histogram, "off_grid_gaps": 0},
                "latency_ms": {"median": 50.0},
                "per_message": {"stamp_s": stamps, "latency_ms": latencies},
            },
            {
                "topic": measure_cloud_timing.DEPTH, "rate_hz": 30.0,
                "content": {"depth": depth}, "content_last": {"depth": depth},
                "per_message": {
                    "stamp_s": depth_stamps, "latency_ms": [6.0] * len(depth_stamps)},
            },
        ],
    }


class TestPooling(unittest.TestCase):
    """The pooled figures are the histogram's, worked out by hand."""

    def test_gaps_are_pooled_over_runs(self):
        report = measure_cloud_timing.pooled([
            probe_result([1, 1, 2, 3], [50.0, 51.0, 49.0, 50.0, 50.0], 0.2),
            probe_result([1, 4, 2, 11], [52.0] * 5, 0.3),
        ])
        cloud = report[measure_cloud_timing.CLOUD]
        self.assertEqual(cloud["frames_per_gap"], {"1": 3, "2": 2, "3": 1, "4": 1, "11": 1})
        self.assertEqual(cloud["gaps"], 8)
        # Mean (1+1+2+3+1+4+2+11) / 8 = 3.125 frames of 33 ms.
        self.assertEqual(cloud["gap_frames"]["mean"], 3.125)
        self.assertEqual(cloud["gap_frames"]["longest"], 11)
        self.assertEqual(cloud["gap_frames"]["median"], 2)
        self.assertAlmostEqual(
            cloud["rate_hz_pooled"], measure_cloud_timing.FRAME_RATE_HZ / 3.125, places=2)
        self.assertAlmostEqual(cloud["worst_gap_s"], 11 / 30.30303, places=2)

    def test_the_depth_image_figures(self):
        report = measure_cloud_timing.pooled([
            probe_result([1, 2], [50.0] * 3, 0.2), probe_result([1, 2], [50.0] * 3, 0.3)])
        depth = report[measure_cloud_timing.DEPTH]
        self.assertEqual(depth["latency_ms_median"], 6.0)
        self.assertAlmostEqual(depth["rate_hz_on_stamps"], 30.303, places=2)
        self.assertAlmostEqual(depth["valid_fraction"], 0.25)
        self.assertAlmostEqual(depth["nan_fraction"], 0.75)
        self.assertEqual(depth["valid_fraction_range"], [0.2, 0.3])

    def test_the_stationary_group_is_pooled_on_stamps(self):
        stamps = [20.0 + 0.1 * index for index in range(41)]
        scan = {
            "topic": "/scan", "message_count": 41, "publisher_count": 1,
            "publisher_qos": [{"reliability": "BEST_EFFORT"}],
            "content": {"scan_time": 0.1343, "time_increment": 0.0},
            "per_message": {"stamp_s": stamps, "latency_ms": [140.0] * 41},
        }
        imu = {
            "topic": "/imu/data", "message_count": 41, "publisher_count": 1,
            "publisher_qos": [{"reliability": "RELIABLE"}],
            "series": {"gyro_x": {"count": 41, "stddev": 0.005}},
            "per_message": {"stamp_s": stamps, "latency_ms": [3.0] * 41},
        }
        absent = {
            "topic": "/imu/mag", "message_count": 0, "publisher_count": 0,
            "publisher_qos": [],
        }
        result = {"topics": [scan, imu, absent]}
        report = measure_cloud_timing.pooled_stationary([result, result])
        self.assertEqual(report["/scan"]["rate_hz_on_stamps"], 10.0)
        self.assertEqual(report["/scan"]["period_ms_median"], 100.0)
        self.assertEqual(report["/scan"]["latency_ms_median"], 140.0)
        self.assertEqual(report["/scan"]["reliability"], ["BEST_EFFORT"])
        self.assertEqual(report["/scan"]["scan_time"], [0.1343])
        self.assertEqual(report["/imu/data"]["series_stddev"], {"gyro_x": 0.005})
        self.assertEqual(report["/imu/data"]["messages"], 82)
        self.assertEqual(report["/imu/mag"], {"runs": 2, "messages": 0, "absent": True})

    def test_the_stationary_group_runs_the_non_camera_probe_groups(self):
        groups = measure_cloud_timing.GROUPS
        self.assertEqual(
            groups["stationary"]["probe_groups"], ("lidar", "imu", "odom", "health"))
        self.assertNotIn("camera", groups["stationary"]["probe_groups"])

    def test_percentile_is_the_nearest_rank_at_or_above(self):
        values = list(range(1, 11))
        self.assertEqual(measure_cloud_timing.percentile(values, 0.95), 10)
        self.assertEqual(measure_cloud_timing.percentile(values, 0.5), 5)
        self.assertEqual(measure_cloud_timing.percentile(values, 0.0), 1)


class TestProbeFetch(unittest.TestCase):
    """The probe comes from a pinned commit by git show and takes only the committed patch."""

    def test_a_short_or_moving_reference_is_refused(self):
        for reference in ("468662c", "main", "origin/main", "468662ca25a5"):
            with self.subTest(reference=reference):
                with self.assertRaises(ValueError):
                    measure_cloud_timing.fetch_probe(
                        PHYSICAL_REPOSITORY, reference, tempfile.mkdtemp())

    def test_the_pin_is_a_full_sha(self):
        self.assertRegex(measure_cloud_timing.PHYSICAL_PIN, r"^[0-9a-f]{40}$")

    def test_the_patch_touches_only_the_clock(self):
        patch = measure_cloud_timing.PATCH.read_text(encoding="utf-8")
        added = [line for line in patch.splitlines() if line.startswith("+") and
                 not line.startswith("+++")]
        joined = "\n".join(added)
        self.assertIn("use_sim_time", joined)
        self.assertIn("get_clock().now()", joined)
        self.assertEqual(len(added), 7, joined)

    def test_the_patch_applies_to_the_pinned_probe_without_touching_the_checkout(self):
        if not (PHYSICAL_REPOSITORY / ".git").exists():
            self.skipTest("no local physical_rosmaster checkout")
        head_before = (PHYSICAL_REPOSITORY / ".git" / "HEAD").read_text()
        with tempfile.TemporaryDirectory() as work:
            probe = measure_cloud_timing.fetch_probe(
                PHYSICAL_REPOSITORY, measure_cloud_timing.PHYSICAL_PIN, work)
            text = probe.read_text(encoding="utf-8")
            self.assertTrue((Path(work) / "physical_contract_probe.py").is_file())
        self.assertIn('Parameter("use_sim_time", Parameter.Type.BOOL, True)', text)
        self.assertIn("receipt = self.get_clock().now().nanoseconds * 1e-9", text)
        self.assertNotIn("receipt = time.time()", text)
        # The header of the copy is physical_rosmaster's own.
        self.assertIn("Apache License, Version 2.0", text[:1200])
        self.assertEqual((PHYSICAL_REPOSITORY / ".git" / "HEAD").read_text(), head_before)


class TestSimulatorRunner(unittest.TestCase):
    """The runner cleans up with anchored patterns and kills by process, not by name."""

    def test_every_cleanup_pattern_is_anchored_by_a_bracketed_first_character(self):
        for pattern in sim_run.CLEANUP_PATTERNS:
            with self.subTest(pattern=pattern):
                self.assertRegex(pattern, r"^\[[^\]]\]")

    def test_no_pattern_can_reach_the_desktop_or_the_browser(self):
        for pattern in sim_run.CLEANUP_PATTERNS:
            with self.subTest(pattern=pattern):
                for word in ("Xvfb", "xvfb", "Xwayland", "firefox", "Xorg"):
                    self.assertNotIn(word, pattern)
                self.assertNotEqual(pattern, "ign")
        # Each is a phrase that only a simulator process line has.
        self.assertTrue(all(re.search(r"[ /]", pattern) for pattern in sim_run.CLEANUP_PATTERNS))

    def test_pkill_is_used_only_with_those_patterns(self):
        source = Path(sim_run.__file__).read_text(encoding="utf-8")
        # One call, looping over CLEANUP_PATTERNS.
        self.assertEqual(source.count('"pkill"'), 1)
        self.assertNotRegex(source, r"killall|pkill[^\n]*[Xx]vfb")

    def test_a_session_lists_its_own_processes(self):
        found = sim_run.session_processes(os.getsid(0))
        self.assertIn(os.getpid(), found)


if __name__ == "__main__":
    unittest.main()
