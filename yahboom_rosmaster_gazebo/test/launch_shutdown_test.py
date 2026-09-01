#!/usr/bin/env python3
"""Focused unit tests for dependency-aware simulator shutdown helpers."""

import importlib.util
from pathlib import Path
import signal
from types import SimpleNamespace
import unittest
from unittest.mock import patch


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "rosmaster_gazebo_fortress.launch.py"
)
SPEC = importlib.util.spec_from_file_location("rosmaster_gazebo_fortress", LAUNCH_FILE)
LAUNCH_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH_MODULE)


class RecordingLogger:
    """Collect helper logs without depending on launch's logging backend."""

    def __init__(self):
        self.debug_messages = []
        self.info_messages = []
        self.warning_messages = []

    def debug(self, message):
        self.debug_messages.append(message)

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)


class FakeProcess:
    """Expose the two launch-process properties used by the helpers."""

    def __init__(self, return_code=None, process_details=None):
        self.return_code = return_code
        self.process_details = process_details


class TestLaunchShutdown(unittest.TestCase):
    """Require the image consumer to stop before Gazebo is torn down."""

    def test_proc_unavailable_probes_live_pid(self):
        process = FakeProcess(process_details={"pid": 123})

        with (
            patch("builtins.open", side_effect=FileNotFoundError),
            patch.object(LAUNCH_MODULE.os.path, "isdir", return_value=False),
            patch.object(LAUNCH_MODULE.os, "kill") as kill,
        ):
            stopped = LAUNCH_MODULE._process_stopped(process)

        self.assertFalse(stopped)
        kill.assert_called_once_with(123, 0)

    def test_proc_unavailable_detects_missing_pid(self):
        process = FakeProcess(process_details={"pid": 123})

        with (
            patch("builtins.open", side_effect=FileNotFoundError),
            patch.object(LAUNCH_MODULE.os.path, "isdir", return_value=False),
            patch.object(
                LAUNCH_MODULE.os, "kill", side_effect=ProcessLookupError),
        ):
            stopped = LAUNCH_MODULE._process_stopped(process)

        self.assertTrue(stopped)

    def test_unstarted_bridge_is_safe(self):
        logger = RecordingLogger()
        bridge = FakeProcess()

        with patch.object(LAUNCH_MODULE.os, "kill") as kill:
            LAUNCH_MODULE._stop_image_bridge(bridge, logger)

        kill.assert_not_called()
        self.assertTrue(any("not started" in message for message in logger.debug_messages))

    def test_stopped_bridge_is_ignored(self):
        logger = RecordingLogger()
        bridge = FakeProcess(return_code=0, process_details={"pid": 123})

        with patch.object(LAUNCH_MODULE.os, "kill") as kill:
            LAUNCH_MODULE._stop_image_bridge(bridge, logger)

        kill.assert_not_called()

    def test_bridge_receives_sigint_and_is_awaited(self):
        logger = RecordingLogger()
        bridge = FakeProcess(process_details={"pid": 123})

        with (
            patch.object(
                LAUNCH_MODULE,
                "_process_stopped",
                side_effect=(False, False, True),
            ),
            patch.object(LAUNCH_MODULE.os, "kill") as kill,
            patch.object(
                LAUNCH_MODULE.time,
                "monotonic",
                side_effect=(0.0, 0.0, 0.1),
            ),
            patch.object(LAUNCH_MODULE.time, "sleep"),
        ):
            LAUNCH_MODULE._stop_image_bridge(bridge, logger)

        kill.assert_called_once_with(123, signal.SIGINT)
        self.assertTrue(any("stopped before" in message for message in logger.info_messages))
        self.assertEqual(logger.warning_messages, [])

    def test_bridge_wait_is_bounded(self):
        logger = RecordingLogger()
        bridge = FakeProcess(process_details={"pid": 123})

        with (
            patch.object(LAUNCH_MODULE, "_process_stopped", return_value=False),
            patch.object(LAUNCH_MODULE.os, "kill") as kill,
            patch.object(LAUNCH_MODULE.time, "monotonic", return_value=1.0),
        ):
            LAUNCH_MODULE._stop_image_bridge(bridge, logger, timeout=0.0)

        kill.assert_called_once_with(123, signal.SIGINT)
        self.assertTrue(any("did not stop" in message for message in logger.warning_messages))

    def test_bridge_is_stopped_before_gazebo_service_request(self):
        calls = []
        context = SimpleNamespace(environment={})
        gazebo = FakeProcess(process_details={"pid": 456})
        bridge = FakeProcess(process_details={"pid": 123})

        def record_bridge_stop(*args):
            del args
            calls.append("image_bridge")

        def record_gazebo_stop(*args, **kwargs):
            del args, kwargs
            calls.append("gazebo")
            return SimpleNamespace(
                returncode=0, stdout="data: true", stderr="")

        with (
            patch.object(
                LAUNCH_MODULE, "_stop_image_bridge",
                side_effect=record_bridge_stop,
            ),
            patch.object(LAUNCH_MODULE.shutil, "which", return_value="/usr/bin/ign"),
            patch.object(
                LAUNCH_MODULE.subprocess, "run",
                side_effect=record_gazebo_stop,
            ),
            patch.object(LAUNCH_MODULE, "_process_stopped", return_value=True),
        ):
            LAUNCH_MODULE._request_gazebo_stop(None, context, gazebo, bridge)

        self.assertEqual(calls, ["image_bridge", "gazebo"])


if __name__ == "__main__":
    unittest.main()
