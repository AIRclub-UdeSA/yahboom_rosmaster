# Running Donatello on a Mac (Apple Silicon)

Everything you need to simulate the ROSMASTER X3 robot on an M-series MacBook,
inspect its LiDAR, and drive it with mecanum keyboard controls.

You do not need to install ROS 2. One tool (`pixi`) sets up the whole
environment, and every command below is run from **inside this folder**.

## Get started

New to this? **[SETUP.md](SETUP.md) is the full walkthrough** — Homebrew, pixi,
cloning, and your first drive, assuming no ROS experience. The short version:

```bash
brew install pixi          # once per machine
cd yahboom_robostack_M_silycon

./run setup                # downloads ROS 2 + Gazebo (once)
./run build                # compiles the robot packages
./run sim                  # opens Gazebo and RViz
```

Two windows open: **Gazebo**, showing the robot in a walled room, and **RViz**,
showing what the robot itself perceives.

Leave that terminal running. Open a second one and drive:

```bash
cd yahboom_robostack_M_silycon
./run teleop
```

Keys: `i` forward, `,` backward, `j`/`l` turn, `k` stop. Press `b` for holonomic
mode, then hold **Shift** with the movement keys to slide sideways — this is a
mecanum robot, so it can strafe without turning.

When you're done, `./run stop`.

## All commands

| Command | What it does |
| --- | --- |
| `./run setup` | install the environment (first time only) |
| `./run build` | compile the robot packages |
| `./run sim` | Gazebo + RViz |
| `./run sim-headless` | no windows, useful for tests |
| `./run teleop` | drive from the keyboard |
| `./run stop` | shut everything down |
| `./run doctor` | check the setup and report problems |

## What the robot publishes

| Topic | Meaning |
| --- | --- |
| `/scan` | LiDAR, 1080 beams over 360°, 0.25–12 m |
| `/odom` | position and velocity estimate |
| `/tf`, `/tf_static` | coordinate frames |
| `/cmd_vel` | velocity commands (publish here to drive it) |
| `/clock` | simulation time |

Inspect them with `cd .. && pixi run -e classic bash`, then ordinary
`ros2 topic list`, `ros2 topic echo /scan`, and so on.

## Two things this simulation does not do

**The wheels do not spin, and walls do not stop the robot.** The plugin that
gives the robot its sideways motion drives the body directly instead of turning
the wheels, so wheel rotation is not simulated and a collision will not physically
halt the base.

**There is no camera.** macOS renders Gazebo camera images blank, so the RGB-D
camera is available only in the Linux Fortress simulation.

If you need exact mecanum wheel physics, use the Gazebo Fortress backend on
Linux (`pixi run sim` from the repository root) — that is the reference
simulation this one stands in for.

## Why this folder exists

Gazebo Fortress, which the rest of this repository targets, sets up its renderer
on a background thread. macOS only allows windows to be created on the main
thread, so on a Mac Fortress cannot open its GUI and cannot run any sensor that
needs rendering — the LiDAR and camera would crash the simulator. The upstream
fix only exists in newer Gazebo and was never backported.

Gazebo Classic, used here, opens its window normally, and its LiDAR works by
asking the physics engine where the walls are instead of drawing them. That is
why `/scan` works on a Mac.

## Contents

```
run                     the only command you need
launch/simulation.launch.py   Gazebo + RViz
worlds/                 the room the robot drives in
rviz/                   RViz layout
scripts/activate.sh     macOS environment fixes, applied automatically
scripts/doctor.sh       the setup checker behind ./run doctor
```

The robot model itself lives in `../yahboom_rosmaster_description`, shared with
the Linux simulation; a `sim_backend:=classic` argument selects the Mac-specific
plugins. The environment is defined in `../pixi.toml`.

## If something goes wrong

Run `./run doctor` first — it checks the environment, the build, and the Gazebo
plugins, and tells you which step to repeat.

**A window doesn't appear, or a command hangs.** A previous run probably left
processes behind. `./run stop`, then try again.

**Nothing publishes / nodes can't see each other.** A Mac exposes around twenty
network interfaces and ROS can pick different ones for different programs. This
folder pins everything to loopback automatically; if you changed
`ROS_LOCALHOST_ONLY`, set it back to `1`.

**`./run build` fails.** Make sure `./run setup` finished. If it still fails,
`cd .. && pixi run clean-classic`, then build again.
