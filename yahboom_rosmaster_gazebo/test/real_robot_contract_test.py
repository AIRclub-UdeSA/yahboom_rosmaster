#!/usr/bin/env python3
"""Keep the simulator/physical parity ledger internally consistent."""

import ast
from pathlib import Path
import re
import sys
import unittest

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / "scripts"))

from cloud_timing import gap_statistics, mean_rate_floor  # noqa: E402
from real_robot_contract import (  # noqa: E402
    OPEN_STEPS,
    RealRobotContract,
    SOURCE_PATH,
    default_path,
    is_range,
    values_agree,
)
from sensor_profiles import SOURCE_PATH as PROFILES_PATH  # noqa: E402
from sensor_profiles import load_sensor_profile  # noqa: E402

CONTRACT = RealRobotContract.load(SOURCE_PATH)
COMMIT = re.compile(r"^[0-9a-f]{40}$")
# physical_rosmaster main after #44 and #45 merged; the commit #43 step 7 re-pinned to.
PHYSICAL_PIN = "468662ca25a52515a218dd944fc031ca85266244"
CAMERA_TOPICS = (
    "/cam_1/color/image_raw",
    "/cam_1/depth/image_raw",
    "/cam_1/color/camera_info",
    "/cam_1/depth/camera_info",
)
SENSOR_CONTRACT_LAUNCH = PACKAGE_DIR / "test" / "sensor_contract.launch.py"


def probe_sample_count():
    """Return sensor_contract.launch.py's default `samples` probe window."""
    tree = ast.parse(SENSOR_CONTRACT_LAUNCH.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "DeclareLaunchArgument"
                and node.args
                and isinstance(node.args[0], ast.Constant)
                and node.args[0].value == "samples"):
            keywords = {keyword.arg: keyword.value for keyword in node.keywords}
            return int(ast.literal_eval(keywords["default_value"]))
    raise AssertionError(f"{SENSOR_CONTRACT_LAUNCH} no longer declares 'samples'")


def keys(node):
    """Yield every mapping key below a node."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield key
            yield from keys(value)
    elif isinstance(node, list):
        for value in node:
            yield from keys(value)


class TestRealRobotContract(unittest.TestCase):
    """Reject parity flags that disagree with the values they describe."""

    def test_parity_flags_agree_with_physical_values(self):
        self.assertEqual(CONTRACT.parity_errors(), [])

    def test_value_comparison_is_exact_unless_a_tolerance_is_given(self):
        self.assertTrue(values_agree(0.0714, 0.0714))
        self.assertFalse(values_agree(0.169, 0.170))
        self.assertTrue(values_agree(0.169, 0.170, tolerance=0.001))
        self.assertTrue(values_agree([1, "a"], [1, "a"]))
        self.assertFalse(values_agree([1, 2], [1, 2, 3]))
        self.assertTrue(values_agree({"x": [0.0]}, {"x": [0.0]}))
        self.assertIsNone(values_agree({"x": 1}, {"y": 1}))
        self.assertIsNone(values_agree(5.0, [2.8, 10.9]))
        self.assertIsNone(values_agree(True, 1))
        self.assertIsNone(values_agree(8.0, None))

    def test_a_physical_range_holds_a_number_inside_it_and_nothing_else(self):
        physical = {"min": -0.013, "max": -0.011}
        self.assertTrue(is_range(physical))
        self.assertTrue(values_agree(-0.012, physical))
        self.assertTrue(values_agree(-0.013, physical))
        self.assertTrue(values_agree(-0.011, physical))
        self.assertFalse(values_agree(-0.0135, physical))
        self.assertFalse(values_agree(0.0, physical))
        # A tolerance widens both ends.
        self.assertTrue(values_agree(-0.0135, physical, tolerance=0.001))
        # Not a number, or not a range: nothing to verify.
        self.assertIsNone(values_agree([-0.012, -0.012], physical))
        self.assertIsNone(values_agree(True, physical))
        self.assertIsNone(values_agree("-0.012", physical))
        self.assertIsNone(values_agree(-0.012, [-0.013, -0.011]))
        self.assertFalse(is_range({"min": -0.011, "max": -0.013}))
        self.assertFalse(is_range({"min": 0.0, "max": 1.0, "mean": 0.5}))
        self.assertFalse(is_range({"min": True, "max": 1.0}))
        self.assertFalse(is_range([0.0, 1.0]))

    def test_a_true_flag_on_a_range_is_verified_and_a_wrong_one_is_caught(self):
        def contract(nominal, matches):
            entry = {"nominal": nominal, "matches_physical": matches}
            if not matches:
                entry["closes_in_step"] = 7
            return RealRobotContract({
                "physical": {"depth": {"scale_error": {"min": -0.013, "max": -0.011}}},
                "simulator": {"depth": {"scale_error": entry}},
            })

        self.assertEqual(contract(-0.012, True).parity_errors(), [])
        self.assertEqual(contract(0.0, False).parity_errors(), [])
        wrong = contract(0.0, True).parity_errors()
        self.assertEqual([error.split(":")[0] for error in wrong], ["depth.scale_error"])
        self.assertIn("differ", wrong[0])
        wrong = contract(-0.012, False).parity_errors()
        self.assertEqual([error.split(":")[0] for error in wrong], ["depth.scale_error"])
        self.assertIn("agree", wrong[0])

    def test_parity_errors_catch_a_wrong_flag(self):
        contract = RealRobotContract({
            "physical": {"camera": {"width": 320, "height": 240}},
            "simulator": {"camera": {
                "width": {"nominal": 424, "matches_physical": True},
                "height": {"nominal": 240, "matches_physical": False,
                           "closes_in_step": 5},
            }},
        })
        errors = contract.parity_errors()
        self.assertEqual(
            [error.split(":")[0] for error in errors],
            ["camera.width", "camera.height"],
            errors,
        )

    def test_parity_errors_fail_a_true_flag_it_cannot_verify(self):
        contract = RealRobotContract({
            "physical": {"cloud": {"rate_hz": [2.83, 10.93], "limit": 0.5}},
            "simulator": {"cloud": {
                "rate_hz": {"nominal": 5.0, "matches_physical": True},
                "limit": {"nominal": {"x": 0.5}, "matches_physical": True},
            }},
        })
        errors = contract.parity_errors()
        self.assertEqual(
            [error.split(":")[0] for error in errors],
            ["cloud.rate_hz", "cloud.limit"],
            errors,
        )
        for error in errors:
            self.assertIn("not comparable", error)
            self.assertIn("matches_physical to false", error)

    def test_parity_errors_accept_a_recorded_gap_that_is_not_comparable(self):
        contract = RealRobotContract({
            "physical": {"cloud": {"rate_hz": [2.83, 10.93], "limit": 0.5}},
            "simulator": {"cloud": {
                "rate_hz": {"nominal": 5.0, "matches_physical": False,
                            "closes_in_step": 6},
                "limit": {"nominal": {"x": 0.5}, "matches_physical": False,
                          "closes_in_step": "out_of_scope",
                          "note": "The physical limit is a scalar."},
            }},
        })
        self.assertEqual(contract.parity_errors(), [])

    def test_provenance_pins_both_commits(self):
        physical = CONTRACT.data["physical"]["provenance"]
        simulator = CONTRACT.data["simulator"]["provenance"]
        self.assertRegex(physical["commit"], COMMIT)
        self.assertEqual(physical["commit"], PHYSICAL_PIN)
        self.assertEqual(physical["read_on"], "2026-09-29")
        self.assertRegex(simulator["measured_commit"], COMMIT)
        self.assertIn("issuecomment", simulator["measurement"])
        for record in simulator.get("step_measurements", []):
            with self.subTest(step=record["step"]):
                self.assertIn(record["step"], OPEN_STEPS)
                self.assertRegex(record["commit"], COMMIT)
                for key in record["keys"]:
                    self.assertIn("measured", CONTRACT.entry(key), key)

    def test_no_measured_key_appears_in_two_records(self):
        """A step that re-measures a key takes it over from the older record."""
        holder = {}
        for record in CONTRACT.data["simulator"]["provenance"]["step_measurements"]:
            for key in record["keys"]:
                with self.subTest(key=key):
                    self.assertNotIn(
                        key, holder,
                        f"in the records of steps {holder.get(key)} and {record['step']}")
                holder[key] = record["step"]

    def test_dotted_keys_are_unambiguous(self):
        for section in ("physical", "simulator"):
            for key in keys(CONTRACT.data[section]):
                with self.subTest(section=section, key=key):
                    self.assertNotIn(".", str(key))

    def test_rate_contracts_bracket_the_nominal_rate(self):
        for key, entry in CONTRACT.entries():
            if "contract" not in entry:
                continue
            with self.subTest(key=key):
                self.assertTrue(key.endswith(".rate_hz"))
                minimum, maximum = entry["contract"]
                self.assertLess(minimum, entry["nominal"])
                self.assertLess(entry["nominal"], maximum)

    def test_cloud_timing_follows_the_default_sensor_profile(self):
        """The ledger's simulator cloud timing is what the physical profile implies."""
        profile = load_sensor_profile(PROFILES_PATH, "point_cloud", "physical")
        statistics = gap_statistics(profile["gap_frames"])
        period = profile["frame_period_s"]
        topic = "topics./cam_1/depth/color/points"
        self.assertAlmostEqual(
            CONTRACT.nominal(f"{topic}.rate_hz"),
            1.0 / (statistics["mean"] * period), delta=0.01)
        self.assertAlmostEqual(
            CONTRACT.nominal(f"{topic}.period_p95_ms"),
            statistics["p95"] * period * 1000.0, delta=0.5)
        self.assertAlmostEqual(
            CONTRACT.nominal(f"{topic}.latency_ms"), profile["latency_s"] * 1000.0)
        self.assertAlmostEqual(
            CONTRACT.nominal(f"{topic}.worst_gap_s"),
            statistics["longest"] * period, delta=0.001)
        nominal = CONTRACT.nominal(f"{topic}.gap_frames")
        self.assertEqual(
            {key: nominal[key] for key in ("median", "p95", "longest")},
            {key: statistics[key] for key in ("median", "p95", "longest")})
        self.assertAlmostEqual(nominal["mean"], statistics["mean"], delta=0.001)

    def test_depth_follows_the_default_sensor_profile(self):
        """The ledger's simulator depth values are what the physical profile implies."""
        profile = load_sensor_profile(PROFILES_PATH, "depth", "physical")
        self.assertEqual(
            CONTRACT.nominal("depth.scale_error"), profile["scale_error"])
        self.assertEqual(
            CONTRACT.nominal("depth.min_range_m"), profile["min_range_m"])
        # The noise string, written out from the profile's three numbers.
        self.assertEqual(
            CONTRACT.nominal("depth.noise_model"),
            "sigma = max({floor:g}, {coefficient:g} * d^{exponent:g}) m".format(
                floor=profile["noise_sigma_floor_m"],
                coefficient=profile["noise_sigma_coefficient"],
                exponent=profile["noise_sigma_exponent"]))
        # The physical side those rows are checked against reads the same.
        self.assertEqual(
            CONTRACT.physical("depth.noise_model"), CONTRACT.nominal("depth.noise_model"))
        self.assertEqual(
            CONTRACT.physical("depth.min_range_m"), profile["min_range_m"])
        # The scale error lies in the physical range (parity_errors checks the
        # flag; this states it) and is the inverse of the doc's 1.012 correction.
        physical = CONTRACT.physical("depth.scale_error")
        self.assertTrue(physical["min"] <= profile["scale_error"] <= physical["max"])
        self.assertAlmostEqual(profile["scale_error"], 1.0 / 1.012 - 1.0, places=6)

    def test_the_ideal_depth_profile_is_undegraded(self):
        profile = load_sensor_profile(PROFILES_PATH, "depth", "ideal")
        self.assertEqual(profile["scale_error"], 0.0)
        self.assertEqual(profile["noise_sigma_floor_m"], 0.0)
        self.assertEqual(profile["noise_sigma_coefficient"], 0.0)
        self.assertEqual(profile["min_range_m"], 0.0)

    def test_render_settings_are_validated_and_not_parity_entries(self):
        self.assertEqual(CONTRACT.settings_errors(), [])
        near = CONTRACT.setting("depth_near_clip_m")
        far = CONTRACT.setting("depth_far_clip_m")
        self.assertLess(near, CONTRACT.nominal("depth.min_range_m"))
        self.assertLess(CONTRACT.nominal("depth.min_range_m"), far)
        # depth.max_range_m is the parity row for the same limit.
        self.assertEqual(CONTRACT.nominal("depth.max_range_m"), far)
        keys = {key for key, _ in CONTRACT.entries()}
        self.assertFalse([key for key in keys if key.startswith("render_settings")])

    def test_a_malformed_render_setting_is_caught(self):
        def errors(settings):
            return RealRobotContract({
                "physical": {"depth": {"min_range_m": 0.6}},
                "simulator": {"render_settings": settings},
            }).settings_errors()

        self.assertEqual(errors({"near": {"nominal": 0.05, "note": "clip"}}), [])
        self.assertTrue(errors({}))
        self.assertTrue(errors({"near": 0.05}))
        self.assertTrue(errors({"near": {"note": "no value"}}))
        self.assertTrue(errors({"near": {"nominal": "0.05"}}))
        self.assertTrue(errors({"near": {"nominal": -1.0}}))
        self.assertTrue(errors({"near": {"nominal": 0.05, "colour": "red"}}))
        self.assertTrue(errors({"near": {"nominal": 0.05, "matches_physical": True}}))
        # A setting named like a physical key is a parity entry in disguise.
        self.assertTrue(errors({"min_range_m": {"nominal": 0.05}}))

    def test_the_cloud_rate_contract_leaves_room_for_a_short_window(self):
        """A mean rate over a few clouds must not fail a correct simulator."""
        minimum, maximum = CONTRACT.rate_bounds("/cam_1/depth/color/points")
        profile = load_sensor_profile(PROFILES_PATH, "point_cloud", "physical")
        # The probe grades a window of `samples` messages, `samples - 1` gaps
        # drawn independently from the physical table (test/sensor_contract
        # .launch.py). Below this rate the window's mean falls only one time
        # in 10,000: the ledger's bound must clear it, or a run this rare
        # would fail a correct simulator. Deriving it here from the sample
        # count and the table, instead of a hand-picked literal, means the
        # two cannot silently drift apart if either changes.
        floor = mean_rate_floor(
            profile["gap_frames"], profile["frame_period_s"], probe_sample_count())
        self.assertLessEqual(minimum, floor)
        self.assertGreaterEqual(maximum, 1.0 / profile["frame_period_s"])

    def test_registered_camera_labels_every_topic_alike(self):
        for section, lookup in (
                ("simulator", CONTRACT.nominal),
                ("physical", CONTRACT.physical)):
            with self.subTest(section=section):
                frames = {
                    lookup(f"topics.{topic}.frame_id")
                    for topic in CAMERA_TOPICS
                }
                self.assertEqual(len(frames), 1, frames)

    def test_superseded_legacy_values_name_their_replacement(self):
        superseded = CONTRACT.data["legacy_bag_audit"]["superseded"]
        for name, record in superseded.items():
            with self.subTest(name=name):
                section, _, key = record["superseded_by"].partition(".")
                self.assertEqual(section, "physical")
                self.assertIsNotNone(CONTRACT.physical(key))

    def test_joint_state_rate_matches_the_controller(self):
        config = yaml.safe_load(
            (PACKAGE_DIR / "config" / "ros2_control.yaml").read_text(
                encoding="utf-8"))
        publish_rate = (
            config["joint_state_broadcaster"]["ros__parameters"]["publish_rate"]
        )
        self.assertEqual(
            publish_rate, CONTRACT.nominal("topics./joint_states.rate_hz"))

    def test_default_path_resolves_to_a_ledger(self):
        self.assertTrue(default_path().is_file())


if __name__ == "__main__":
    unittest.main()
