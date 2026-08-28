# Contributing to yahboom_rosmaster

Thanks for being here. `yahboom_rosmaster` is the ROSMASTER X3 simulator/digital
twin behind AIR Club UdeSA's [Challenge JAR](https://github.com/AIRclub-UdeSA/jar_site) —
every team competing at JAR 2026 (Jornada Argentina de Robótica, Rosario) builds
and tests their robot behavior against this world before it ever touches a
physical robot. It gets better when more people run it, break it, and extend it.

## Getting set up

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Gazebo Fortress 6
- `git`, `colcon`, and `rosdep`

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep ros-humble-ros-gz

mkdir -p ~/rosmaster_ws/src
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/yahboom_rosmaster.git

cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --symlink-install
source install/setup.bash
```

Apple Silicon Macs use a RoboStack/pixi environment instead — see
[macOS (Apple Silicon)](README.md#macos-apple-silicon) in the README. Once
you're built, the [Quick Start](README.md#quick-start) section covers launching
the simulator, driving the robot, and picking a world (including the maze
worlds under [Maze Worlds](README.md#maze-worlds)).

## Good first contributions

The highest-leverage places to jump in, straight from the project's own
[Current Project Status](README.md#current-project-status) and
[Troubleshooting](README.md#troubleshooting) sections:

- 🧩 **Add or improve a maze world.** Only `maze_3_6x6.world` ships a matching
  occupancy map (`maps/maze_3.yaml`) — the other seven maze worlds have none.
  The obstacle cubes in `laberinto_simple_victimas.world` and
  `laberinto_1_victimas.world` are meant to be detected live via camera/LiDAR,
  not pre-mapped, so tooling or challenge worlds around live detection are
  welcome too.
- 🎯 **Help calibrate the drivetrain.** The default `stress` wheel-contact
  profile is deterministic but uncalibrated — it must not be described as
  reproducing the physical ROSMASTER X3 until its contact values are fitted
  against synchronized wheel odometry and external ground truth. The `ideal`
  profile is the current zero-slip baseline.
- 📷 **Help calibrate sensors.** Camera, LiDAR, and IMU output is nominal
  simulation data, not yet validated against the physical robot; IMU
  covariance arrays are all zero (ROS's "unknown," not a measured value).
- 🍏 **Improve macOS / Gazebo Classic parity.** The Apple Silicon backend
  trades away the Gazebo GUI and LiDAR on Fortress for a Classic backend that
  restores both — see [What does not work on macOS](README.md#what-does-not-work-on-macos)
  for the current gap list.
- 🐛 **Fix bugs.** See [Troubleshooting](README.md#troubleshooting) for known
  rough edges (stale workspace overlays, controller/odometry readiness, etc.).

## Repository Layout

| Package / Directory | Contents |
|---------------------|----------|
| `yahboom_rosmaster` | Repository metapackage |
| `yahboom_rosmaster_bringup` | Canonical simulator launch entrypoints and helpers |
| `yahboom_rosmaster_description` | Xacro/URDF, kinematic parameters, visual/collision meshes, and RViz configs |
| `yahboom_rosmaster_gazebo` | Fortress worlds, bridge configs, simulator launch, watchdog, odometry, and test probes |
| `yahboom_rosmaster_msgs` | Custom interface definitions |
| `yahboom_robostack_M_silycon` | macOS Apple Silicon pixi + Gazebo Classic simulation environment |
| `dockerfiles` | Docker container setup (`Dockerfile`, `container.sh`) for isolated Linux simulation |

Read [Simulator Architecture](README.md#simulator-architecture) before touching
the command flow, the bridge, or the watchdog — it documents how `/cmd_vel`
gets from ROS to Gazebo and back, and how odometry/TF are assembled.

## Coding style

- Python nodes are linted with `ament_flake8` and `ament_pep257` across every
  package — keep docstrings and formatting consistent with that.
- ROS 2 launch files follow the existing Python launch API style in
  `yahboom_rosmaster_bringup` and `yahboom_rosmaster_gazebo` — match the
  surrounding structure rather than inventing a new pattern.
- SDF/world/xacro files: match the surrounding indentation and comment
  density. Comment the *why* (a tuned physics value, a workaround for a
  simulator quirk) — not the obvious *what*.
- Prefer small, reviewable commits.

## Pull requests

Changes to `main` must go through a pull request:

1. Create a feature branch (`git checkout -b feature/my-change`) and push it.
2. Before opening the PR, run the [Development Checks](README.md#development-checks)
   from the workspace root — `colcon build`, `colcon test`, `py_compile`,
   `xacro` + `check_urdf`, and `rosdep check` — and keep them green.
3. If you change runtime behavior (a launch arg, a topic, a default profile),
   update the [Current Project Status](README.md#current-project-status)
   section of the README in the same PR.
4. If you add a world, model, or asset, make sure you have the right to
   redistribute it and note its origin (see [Provenance](README.md#provenance)
   for the pattern this repo already follows).
5. Describe what you changed and how you verified it — a screenshot or a
   short recording is welcome for anything visual, matching the media already
   in `docs/media/`.
6. At least 1 approval from another contributor is required before merging.
7. Resolve all review comments before merging.
8. New commits after an approval will require re-approval.
9. Direct pushes to `main` are blocked, including for repo admins.

## Ground rules

- This simulator is what every JAR team calibrates their challenge behavior
  against — don't overstate fidelity. If a profile, sensor model, or world is
  uncalibrated or nominal, say so, the same way the README already does.
- Changes to shared worlds, the robot model, or launch defaults affect every
  team's environment at once. Flag breaking changes clearly in the PR
  description rather than assuming they'll be discovered downstream.
- Be decent to each other. Assume good faith, keep it constructive.
- By contributing, you agree your contributions are licensed under the
  BSD-3-Clause license already declared in each package's `LICENSE` file.
