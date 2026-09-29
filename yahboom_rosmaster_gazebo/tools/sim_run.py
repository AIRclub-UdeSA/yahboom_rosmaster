"""
Start, watch and stop one headless simulator for the measurement tools.

Everything here follows the rules that cost time on this project (AGENTS.md):
leftover processes and Fast DDS shared memory are cleared before every launch
with anchored patterns; the launch is started detached in its own session with
``Popen(start_new_session=True)``, never with a shell ``&``, which would leave it
unable to stop cleanly; and it is stopped with SIGINT, then SIGTERM, and a
process group kill only when it stalls.

The tools are run from a source tree and are not installed as nodes.
"""

import os
from pathlib import Path
import signal
import subprocess
import time

# Bracketed first characters keep each pattern from matching the shell or the
# tool that runs it. ``pkill -f ign`` would also match Firefox and system
# services, so nothing here is a bare word.
CLEANUP_PATTERNS = (
    "[/]usr/bin/ign gazebo",
    "[r]os2 launch",
    "[r]osmaster_ws(_step7)?/install/",
)
LLVMPIPE_ENV = {
    "LIBGL_ALWAYS_SOFTWARE": "1",
    "GALLIUM_DRIVER": "llvmpipe",
    "LP_NUM_THREADS": "4",
}
XVFB_ARGS = "-screen 0 1280x1024x24 -nolisten tcp"
CLOCK_TICKS = os.sysconf("SC_CLK_TCK")
ROS_SETUP = "/opt/ros/humble/setup.bash"


def isolated_environment(domain_id, partition):
    """Return the environment variables that keep this sim to itself."""
    return {
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_DOMAIN_ID": str(domain_id),
        "IGN_PARTITION": partition,
    }


def _ros_shell(command, workspace=None):
    """Return a bash line that sources ROS (and a workspace) and runs ``command``."""
    lines = [f"source {ROS_SETUP}"]
    if workspace is not None:
        lines.append(f"source {Path(workspace) / 'install' / 'setup.bash'}")
    lines.append(command)
    return "\n".join(lines)


def clean_up():
    """Kill leftover simulator processes and clear Fast DDS shared memory."""
    for pattern in CLEANUP_PATTERNS:
        subprocess.run(["pkill", "-9", "-f", pattern], check=False)
    time.sleep(1.0)
    for entry in Path("/dev/shm").glob("fastrtps_*"):
        entry.unlink(missing_ok=True)
    for entry in Path("/dev/shm").glob("sem.fastrtps_*"):
        entry.unlink(missing_ok=True)
    subprocess.run(
        ["bash", "-c", _ros_shell("ros2 daemon stop")],
        check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def session_processes(session_id):
    """Return {pid: (command line, CPU ticks)} for every process of a session."""
    found = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
            fields = stat[stat.rindex(")") + 2:].split()
            if int(fields[3]) != session_id:
                continue
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                errors="replace").strip()
            found[int(entry.name)] = (command, int(fields[11]) + int(fields[12]))
        except (OSError, ValueError, IndexError):
            continue
    return found


class Simulator:
    """One headless simulator launch, started detached in its own session."""

    def __init__(self, workspace, log_path, launch_arguments, environment,
                 render="gpu"):
        self.workspace = Path(workspace)
        self.log_path = Path(log_path)
        self.launch_arguments = list(launch_arguments)
        self.environment = dict(environment)
        self.render = render
        self._process = None

    def _command(self):
        launch = (
            "exec ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py "
            + " ".join(self.launch_arguments))
        if self.render == "llvmpipe":
            # CI's setup, pinned to the four CPUs of its runner. "exec" keeps
            # xvfb-run's child the launch itself.
            launch = (
                f'exec taskset -c 0-3 xvfb-run --auto-servernum '
                f'--server-args="{XVFB_ARGS}" bash -c \'{launch}\'')
        return ["bash", "-c", _ros_shell(launch, self.workspace)]

    def start(self):
        """Clean up, then start the launch detached."""
        clean_up()
        environment = dict(os.environ)
        environment.update(self.environment)
        if self.render == "llvmpipe":
            environment.update(LLVMPIPE_ENV)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.log_path, "w", encoding="utf-8") as log:
            self._process = subprocess.Popen(
                self._command(), stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, env=environment, start_new_session=True)

    @property
    def session_id(self):
        """Return the session (and process group) id of the launch, which is its pid."""
        return self._process.pid

    def processes(self):
        """Return the session's processes with their CPU ticks."""
        return session_processes(self.session_id)

    def alive(self):
        """Return whether the launch is still running."""
        return self._process is not None and self._process.poll() is None

    def stop(self, timeout_s=40.0):
        """
        Stop the launch cleanly and return how it ended.

        SIGINT goes to the ``ros2 launch`` process, as Ctrl-C does. A launch that
        has not exited after ``timeout_s`` is terminated, and then its whole
        session is killed. The session sweep also removes the ``Xvfb`` that a
        killed ``xvfb-run`` would leave behind.
        """
        outcome = "clean"
        if self._process is None:
            return outcome
        launch_pids = [
            pid for pid, (command, _) in self.processes().items()
            if "ros2" in command and "launch" in command
            and "rosmaster_gazebo_fortress" in command and "bash -c" not in command]
        for pid in launch_pids:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self._sim_processes_left():
            time.sleep(0.5)
        if self._sim_processes_left():
            outcome = "stalled"
            for pid in launch_pids:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            time.sleep(5.0)
        for pid in self.processes():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        try:
            self._process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            outcome = "unreaped"
        return outcome

    def _sim_processes_left(self):
        return any(
            "ign gazebo" in command or "camera_adapter" in command
            for command, _ in self.processes().values())
