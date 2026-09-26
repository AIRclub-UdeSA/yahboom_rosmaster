#!/usr/bin/env python3
"""Keep config/sensor_profiles.yaml valid and faithful to its measurement."""

from pathlib import Path
import sys
import tempfile
import unittest

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from sensor_profiles import (  # noqa: E402
    PROFILE_NAMES,
    SENSOR_KEYS,
    SOURCE_PATH,
    default_path,
    load_sensor_profile,
    point_cloud_parameters,
)


def document():
    """Return the shipped profile document, to edit into an invalid one."""
    return yaml.safe_load(SOURCE_PATH.read_text(encoding="utf-8"))


def load_edited(edit, profile="physical"):
    """Load the profile from the shipped document after ``edit`` changed it."""
    data = document()
    edit(data["point_cloud"][profile])
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "sensor_profiles.yaml"
        path.write_text(yaml.safe_dump(data), encoding="utf-8")
        return load_sensor_profile(path, "point_cloud", profile)


class TestShippedProfiles(unittest.TestCase):
    """The file the launch loads is complete and matches the measurement."""

    def test_every_sensor_has_both_profiles(self):
        data = document()
        for sensor in SENSOR_KEYS:
            with self.subTest(sensor=sensor):
                self.assertEqual(set(data[sensor]), set(PROFILE_NAMES))

    def test_every_value_names_its_source(self):
        for profile, values in document()["point_cloud"].items():
            for key, record in values.items():
                with self.subTest(profile=profile, key=key):
                    self.assertTrue(record["source"].strip())

    def test_the_physical_values_are_the_measured_ones(self):
        profile = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")
        self.assertEqual(profile["latency_s"], 0.050)
        self.assertEqual(profile["frame_period_s"], 0.033)
        self.assertEqual(len(profile["gap_frames"]), 26)
        self.assertEqual(max(profile["gap_frames"]), 34)
        self.assertAlmostEqual(sum(profile["gap_frames"].values()), 0.9998)

    def test_the_ideal_profile_delivers_every_frame_undelayed(self):
        profile = load_sensor_profile(SOURCE_PATH, "point_cloud", "ideal")
        self.assertEqual(profile["latency_s"], 0.0)
        self.assertEqual(profile["gap_frames"], {1: 1.0})

    def test_both_profiles_share_one_frame_clock(self):
        clocks = {
            load_sensor_profile(SOURCE_PATH, "point_cloud", name)["frame_period_s"]
            for name in PROFILE_NAMES
        }
        self.assertEqual(len(clocks), 1)

    def test_the_installed_lookup_falls_back_to_the_source_tree(self):
        self.assertTrue(default_path().is_file())

    def test_the_node_parameters_pair_each_gap_with_its_probability(self):
        profile = load_sensor_profile(SOURCE_PATH, "point_cloud", "physical")
        parameters = point_cloud_parameters(profile)
        self.assertEqual(
            dict(zip(parameters["gap_frames"], parameters["gap_probabilities"])),
            profile["gap_frames"])
        self.assertTrue(all(isinstance(gap, int) for gap in parameters["gap_frames"]))
        self.assertEqual(parameters["latency_s"], 0.050)


class TestValidation(unittest.TestCase):
    """A malformed profile stops the launch instead of running wrong."""

    def test_an_unknown_profile_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "Unknown sensor profile 'noisy'"):
            load_sensor_profile(SOURCE_PATH, "point_cloud", "noisy")

    def test_an_unknown_sensor_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "Unknown sensor 'lidar'"):
            load_sensor_profile(SOURCE_PATH, "lidar", "physical")

    def test_a_missing_key_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "missing=\\['latency_s'\\]"):
            load_edited(lambda profile: profile.pop("latency_s"))

    def test_an_unknown_key_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "extra=\\['jitter_s'\\]"):
            load_edited(lambda profile: profile.update(
                jitter_s={"value": 0.001, "source": "made up"}))

    def test_a_value_without_a_source_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "'value' and 'source'"):
            load_edited(lambda profile: profile["latency_s"].pop("source"))
        with self.assertRaisesRegex(RuntimeError, "needs a source"):
            load_edited(lambda profile: profile["latency_s"].update(source="  "))

    def test_a_bare_number_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "'value' and 'source'"):
            load_edited(lambda profile: profile.update(latency_s=0.05))

    def test_a_non_numeric_value_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "must be numeric"):
            load_edited(lambda profile: profile["latency_s"].update(value="fifty"))
        with self.assertRaisesRegex(RuntimeError, "must be numeric"):
            load_edited(lambda profile: profile["latency_s"].update(value=True))

    def test_out_of_range_values_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "at least 0.0"):
            load_edited(lambda profile: profile["latency_s"].update(value=-0.01))
        with self.assertRaisesRegex(RuntimeError, "greater than 0.0"):
            load_edited(lambda profile: profile["frame_period_s"].update(value=0.0))
        with self.assertRaisesRegex(RuntimeError, "at least 0.0"):
            load_edited(lambda profile: profile["latency_s"].update(value=float("nan")))

    def test_a_malformed_gap_table_is_rejected(self):
        def with_gaps(gaps):
            return lambda profile: profile["gap_frames"].update(value=gaps)

        with self.assertRaisesRegex(RuntimeError, "mapping of frames"):
            load_edited(with_gaps({}))
        with self.assertRaisesRegex(RuntimeError, "whole number"):
            load_edited(with_gaps({0: 0.5, 1: 0.5}))
        with self.assertRaisesRegex(RuntimeError, "whole number"):
            load_edited(with_gaps({1.5: 1.0}))
        with self.assertRaisesRegex(RuntimeError, "greater than 0.0"):
            load_edited(with_gaps({1: 1.0, 2: 0.0}))
        with self.assertRaisesRegex(RuntimeError, "not 1"):
            load_edited(with_gaps({1: 0.5, 2: 0.3}))


if __name__ == "__main__":
    unittest.main()
