#!/usr/bin/env python3
"""Shared post-shutdown exit-code contract for the simulator launch tests.

The launch file force-kills Gazebo with SIGKILL when its service-requested
clean stop stalls past GAZEBO_CLEAN_STOP_TIMEOUT, because the SIGINT fallback
re-enters an already in-flight SensorsPrivate::Stop() and segfaults (issue
#31). That escalation is deliberate, so the Gazebo server is allowed to report
the resulting -9. Every other process must still exit 0, which is what keeps
these tests able to catch stray signals and orphan-prone exits.
"""

EXIT_OK = 0
GAZEBO_FORCED_STOP_EXIT_CODE = -9

# The server runs as `ruby <ign> gazebo ...`, so launch names it `ruby-N`.
GAZEBO_SERVER_NAME_PREFIX = "ruby"


def allowable_exit_codes(process_name):
    """Return the exit codes tolerated for a process, given its launch name."""
    if process_name.startswith(GAZEBO_SERVER_NAME_PREFIX):
        return (EXIT_OK, GAZEBO_FORCED_STOP_EXIT_CODE)
    return (EXIT_OK,)


def assert_clean_shutdown(proc_info):
    """Assert every launched process exited cleanly, or was force-killed Gazebo."""
    for info in proc_info:
        allowed = allowable_exit_codes(info.process_name)
        assert info.returncode in allowed, (
            f"Proc {info.process_name} exited with code {info.returncode}; "
            f"expected one of {list(allowed)}")
