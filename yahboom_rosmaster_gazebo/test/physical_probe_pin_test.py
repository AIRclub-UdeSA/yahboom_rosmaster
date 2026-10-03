#!/usr/bin/env python3
"""Keep the physical contract probe's pin, fetch and launch wiring honest (#43 step 9)."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_DIR = PACKAGE_DIR.parent
sys.path.insert(0, str(PACKAGE_DIR / "tools"))
sys.path.insert(0, str(PACKAGE_DIR / "test"))

import measure_cloud_timing  # noqa: E402
import physical_probe  # noqa: E402
from sim_timing import PHYSICAL_PROBE_PARAMETERS  # noqa: E402

WORKFLOW = REPOSITORY_DIR / ".github" / "workflows" / "simulator-contracts.yml"
LAUNCH = PACKAGE_DIR / "test" / "sensor_contract.launch.py"


def fetch_pinned_probe(work_dir):
    """
    Return (path, blob) of the probe at the ledger's pin.

    Where the checkout is missing a developer can skip. CI cannot: it fails, so
    removing the fetch step (or breaking the pin) turns the gate red.
    """
    try:
        return physical_probe.fetch_contract_probe(
            physical_probe.repository(), physical_probe.ledger_pin(), work_dir)
    except physical_probe.ProbeUnavailable as error:
        if os.environ.get("GITHUB_ACTIONS") or os.environ.get("CI"):
            raise
        raise unittest.SkipTest(f"no physical_rosmaster checkout here: {error}")


class TestPin(unittest.TestCase):
    """The ledger holds the one pin, and everything reads it."""

    def test_the_ledger_pin_is_a_full_sha(self):
        self.assertRegex(physical_probe.ledger_pin(), r"^[0-9a-f]{40}$")

    def test_the_measurement_tool_reads_the_ledger_pin(self):
        self.assertEqual(
            measure_cloud_timing.PHYSICAL_PIN, physical_probe.ledger_pin())

    def test_a_short_or_moving_reference_is_refused(self):
        for reference in (
                "fa56a87", "main", "origin/main", "", None,
                "FA56A87BEB186FAB7545E650265EB43792ACF827"):
            with self.subTest(reference=reference):
                with self.assertRaises(ValueError):
                    physical_probe.require_full_sha(reference)
                with self.assertRaises(ValueError):
                    physical_probe.fetch_contract_probe(
                        PACKAGE_DIR, reference, tempfile.mkdtemp())

    def test_a_missing_checkout_or_commit_is_an_error_not_a_skip(self):
        with tempfile.TemporaryDirectory() as empty:
            with self.assertRaises(physical_probe.ProbeUnavailable):
                physical_probe.fetch_contract_probe(
                    Path(empty) / "nowhere", physical_probe.ledger_pin(), empty)
            subprocess.run(["git", "init", "-q", empty], check=True)
            with self.assertRaises(physical_probe.ProbeUnavailable) as raised:
                physical_probe.fetch_contract_probe(
                    empty, physical_probe.ledger_pin(), empty)
            self.assertIn(physical_probe.ledger_pin(), str(raised.exception))


class TestFetchedProbe(unittest.TestCase):
    """The probe at the pin is the one that grades the simulator."""

    def test_the_fetched_blob_declares_the_target_parameter(self):
        with tempfile.TemporaryDirectory() as work:
            path, blob = fetch_pinned_probe(work)
            text = path.read_text(encoding="utf-8")
        self.assertRegex(blob, r"^[0-9a-f]{40}$")
        self.assertIn('declare_parameter("target"', text)
        self.assertIn('"simulator": Target(', text)
        # physical_rosmaster's own header stays on the copy.
        self.assertIn("Apache License, Version 2.0", text[:1200])


class TestWiring(unittest.TestCase):
    """The launch tests and the workflow run the probe the way the note says."""

    def test_the_probe_is_started_for_the_simulator_on_sim_time_with_ten_samples(self):
        self.assertEqual(PHYSICAL_PROBE_PARAMETERS["target"], "simulator")
        self.assertIs(PHYSICAL_PROBE_PARAMETERS["use_sim_time"], True)
        # Five samples false-fail the cloud's 3 Hz floor about 1.8% of the time.
        self.assertGreaterEqual(PHYSICAL_PROBE_PARAMETERS["samples"], 10)
        self.assertGreater(PHYSICAL_PROBE_PARAMETERS["timeout"], 0.0)

    def test_the_launch_test_starts_it_when_the_sensor_probe_exits(self):
        text = LAUNCH.read_text(encoding="utf-8")
        for needed in (
                "OnProcessExit", "target_action=probe", "physical_contract_probe",
                "test_physical_contract_passes", "physical_probe_command()",
                "assertExitCodes(proc_info, process=physical_probe)"):
            self.assertIn(needed, text)

    def test_the_workflow_fetches_the_ledgers_pin_into_the_checkout_the_tests_read(self):
        workflow = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
        job = workflow["jobs"]["headless-contracts"]
        steps = job["steps"]
        reads = [step for step in steps if step.get("id") == "physical_pin"]
        self.assertEqual(len(reads), 1)
        self.assertIn("real_robot_contract.yaml", reads[0]["run"])
        self.assertIn("[0-9a-f]{40}", reads[0]["run"])
        fetches = [
            step for step in steps
            if step.get("with", {}).get("repository") == "AIRclub-UdeSA/physical_rosmaster"]
        self.assertEqual(len(fetches), 1)
        self.assertEqual(
            fetches[0]["with"]["ref"], "${{ steps.physical_pin.outputs.sha }}")
        self.assertIn(
            "tools/physical_contract_probe.py", fetches[0]["with"]["sparse-checkout"])
        gate = [step for step in steps if "test_simulator_contracts.sh" in step.get("run", "")]
        self.assertEqual(len(gate), 1)
        self.assertIn(
            "physical_rosmaster",
            gate[0]["env"][physical_probe.ENVIRONMENT_VARIABLE])
        # The fetch comes after the build, so colcon never scans it, and before the gate.
        build = [step for step in steps if step.get("name") == "Build supported packages"]
        self.assertEqual(len(build), 1)
        self.assertLess(steps.index(build[0]), steps.index(fetches[0]))
        self.assertLess(steps.index(fetches[0]), steps.index(gate[0]))


if __name__ == "__main__":
    unittest.main()
