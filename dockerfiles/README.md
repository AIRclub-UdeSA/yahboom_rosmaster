# Docker (Containerized Linux Simulation)

Runs the simulator in an isolated `ros:humble-ros-base-jammy` container so the
host does not need Ubuntu 22.04 or ROS 2 Humble installed natively. This is the
path validated by
[yahboom_rosmaster#18](https://github.com/AIRclub-UdeSA/yahboom_rosmaster/issues/18)
and is the basis for the optional Docker guide on
[jar_site](https://github.com/AIRclub-UdeSA/jar_site).

## Requirements

- Docker Engine ([official install docs](https://docs.docker.com/engine/install/ubuntu/))
- Your user in the `docker` group (`sudo usermod -aG docker $USER`, then start a
  new login session)
- `xauth` on the host, for GUI (X11) forwarding
- The NVIDIA Container Toolkit, only if you have an NVIDIA GPU and want
  hardware-accelerated rendering through it

The host's Linux distribution and ROS distribution do not matter — the
container brings its own Ubuntu 22.04 / ROS 2 Humble userspace. This flow was
validated from a clean install on **Ubuntu 24.04 with ROS 2 Jazzy on the
host**, Docker Engine 29.8.0, on a machine with an **AMD integrated GPU** (no
NVIDIA hardware was available to validate that path; see
[Limitations](#limitations)).

## Quick Start

```bash
cd dockerfiles

./container.sh start         # build the image and start the container
./container.sh build         # install dependencies and compile the workspace
./container.sh sim           # launch Gazebo + RViz
./container.sh teleop        # drive the robot from another terminal
./container.sh stop          # stop the container
```

Use `./container.sh sim-headless` when no GUI is needed. Run
`./container.sh doctor` to inspect the container, GPU, and X11 connection.

`start` and `stop` preserve the container (and anything `build` installed into
it) across runs. `./container.sh clean` removes the container entirely,
including any packages `rosdep` installed at runtime — run `build` again after
a `clean` before `sim` will work.

## Validated

Every subcommand was exercised end to end from a from-scratch container on the
host/GPU combination above:

- `start` — image builds, container starts, workspace bind-mounts correctly.
- `build` — `rosdep install` resolves and installs every missing dependency,
  `colcon build` compiles all 5 packages.
- `doctor` — correctly reports Docker, container state, GPU, and X11 status;
  its in-container `xdpyinfo` check confirms X11 is actually reachable, not
  just that `DISPLAY` is set.
- `sim-headless` — full launch, every `ros_gz_bridge` topic comes up, sensor
  rates match the [top-level README's contract](../README.md#working-ros-interfaces)
  (`/odom` ~30 Hz, `/scan` ~5 Hz, `/imu/data` ~10 Hz), `joint_state_broadcaster`
  loads and activates.
- `sim` (GUI) — Gazebo and RViz windows open on the host's X server via the
  forwarded `DISPLAY`/`XAUTHORITY`, robot model renders correctly.
- Motion — a direct `/cmd_vel` publish moves the robot (verified via `/odom`
  position deltas), and the 0.5 s watchdog correctly zeroes `/cmd_vel_gz` after
  command silence.
- `teleop` — `ros-humble-teleop-twist-keyboard` resolves and the node starts
  cleanly (interactive keyboard input itself is not testable
  non-interactively).
- GPU passthrough — `/dev/dri` render-node access from inside the container,
  for the no-NVIDIA/Mesa path (see the bugs below; this did not work until
  fixed).
- No-GPU fallback — `container.sh` falls back to `LIBGL_ALWAYS_SOFTWARE=1`
  automatically when `/dev/dri` is absent; not separately exercised on this
  GPU-equipped machine, but the fallback branch was reviewed and is
  straightforward.

## Bugs found and fixed

Three real, reproducible bugs surfaced during this validation, all in
`container.sh`, all now fixed on this branch:

1. **X11 forwarding was silently broken whenever `DISPLAY` was set** (i.e. in
   normal desktop use). `get_display_args()` built a Bash array and returned it
   by piping through `echo "${args[@]}"`, then the caller re-split that string
   on whitespace. Because the array's first element is the literal string
   `-e`, Bash's `echo` builtin consumed it as its own "interpret escapes"
   flag instead of emitting it — silently dropping the first `-e` from the
   output. The result: `docker run ... DISPLAY=:0 -e XAUTHORITY=... "$IMAGE"`
   with `DISPLAY=:0` now a bare positional argument, which `docker` reads as
   the image name: `docker: invalid reference format: repository name
   (library/DISPLAY=) must be lowercase`. Every command that forwards a
   display (`start`, `enter`, `build`, `sim`, `sim-headless`, `teleop`,
   `doctor`) was affected. Fixed by having `get_display_args()` populate a
   caller-provided array via a nameref (`local -n`) instead of round-tripping
   through a string.
2. **`rosdep install` failed on every package not already baked into the
   image** (`ros2-control`, `ros2-controllers`, `gz-ros2-control`,
   `controller-manager`, `topic-tools`, `rqt-robot-steering`,
   `rviz-imu-plugin`, `joint-state-publisher-gui`, `ros2controlcli` — i.e.
   most of what a fresh clone actually needs). The `Dockerfile` correctly ends
   with `rm -rf /var/lib/apt/lists/*` to keep the image small, but
   `install_dependencies()` never ran `apt-get update` afterward inside the
   container, so every `apt-get install` invoked by `rosdep` hit an empty
   package index (`E: Unable to locate package ...`). Fixed by adding
   `sudo apt-get update` before `rosdep update`/`rosdep install`.
3. **GPU render-node access failed with `Permission denied`** on a non-NVIDIA
   (Mesa/AMD, and likely Intel) GPU, even though `/dev/dri` was correctly
   passed through. The container is granted access by adding the *group* that
   owns the render node as a supplementary group
   (`--group-add <group>`), but the script resolved that via
   `find /dev/dri -printf '%g'`, i.e. the group **name** as seen on the host
   (`render`). Docker resolves a `--group-add` name against the *container's*
   `/etc/group`, and the `Dockerfile` creates that group dynamically at build
   time (`groupadd --system render`) — landing on whatever numeric GID happens
   to be next-available at build time, with no relationship to the host's
   actual `render` GID. On this machine the host's `render` group is GID 992,
   but the image's own `render` group is 999, so `--group-add render` added
   the wrong GID and the real device (owned by 992) stayed inaccessible.
   Fixed by using `find /dev/dri -printf '%G'` (the numeric GID) instead —
   Docker adds a numeric `--group-add` value directly as a raw supplementary
   GID, with no dependency on the container having a matching `/etc/group`
   entry at all.

Bug 3 did not stop the Gazebo or RViz windows from opening (RViz's 3D viewport
renders through the forwarded X11 socket, i.e. the *host's* GPU driver, not
`/dev/dri` inside the container), but it silently broke EGL for anything
Gazebo renders through its own process — its own GUI viewport and any
GPU-rendered sensor (camera, depth, GPU LiDAR).

## Known gotchas (not bugs, but worth knowing)

- **`container.sh clean` wipes rosdep-installed packages.** `build` installs
  dependencies into the *container's* writable layer at runtime, not into the
  image. `clean` removes the container, discarding that layer; the next
  `start` begins from the original image again, missing everything `build`
  added. Re-run `build` after any `clean`. Prefer `stop`/`start` (which
  preserves the container) unless you specifically want to reset it.
- **Stale processes from a previous run corrupt the next one.** If a
  container is reused without a clean `stop`/`start` between simulation runs
  (e.g. a launch is killed with `pkill` instead of stopping the container),
  leftover `robot_state_publisher` and Gazebo/RViz processes keep running
  alongside a newly launched set. Symptoms: RViz repeatedly logging "jump
  back in time" / "Resetting RViz", a flickering or broken robot model, or
  literally two Gazebo/RViz windows. This mirrors the
  [native-install troubleshooting note](../README.md#troubleshooting) about
  stale processes breaking discovery — the fix is the same: stop the
  container fully before relaunching, don't just kill the launch process.

## Limitations

- **No NVIDIA GPU was available to validate the NVIDIA Container Toolkit
  path.** `container.sh` detects an NVIDIA GPU and passes `--gpus all`
  automatically when the toolkit is installed, but that branch is unverified
  by this pass — validate before advertising it as supported.
- Validated on a single machine, a single Docker Engine version (29.8.0), and
  a single non-NVIDIA GPU vendor (AMD). Other Mesa-based GPUs (Intel) should
  behave the same way given bug 3's fix, but were not directly tested.
- Interactive keyboard teleoperation itself was not exercised non-interactively
  — only that the package resolves and the node starts. Combined with the
  `/cmd_vel` motion validation above, the underlying command path is
  confirmed; the keyboard capture loop is not.
- This validates the containerized workflow, not the simulator itself — the
  [top-level README's "Current Project Status"](../README.md#current-project-status)
  and its uncalibrated sensor/drivetrain caveats apply identically inside the
  container.
