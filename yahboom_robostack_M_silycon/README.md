# Running the ROSMASTER X3 on a Mac (Apple Silicon)

Everything you need to simulate the robot on an M-series MacBook: drive it,
see its LiDAR, build a map, and let it navigate on its own.

You do not need to install ROS 2. One tool (`pixi`) sets up the whole
environment, and every command below is run from **inside this folder**.

## Get started

```bash
brew install pixi          # once per machine
cd yahboom_robostack_M_silycon

./run setup                # downloads ROS 2 + Gazebo (~3 GB, once)
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

## Mapping and navigation

With the simulation running, in a second terminal:

```bash
./run slam       # builds a map as you drive
./run nav2       # builds a map AND navigates on its own
```

**Drive around the room first.** The robot can only navigate through space it has
already seen with its LiDAR, so open a teleop terminal and tour the room before
sending a goal. Watch the map fill in inside RViz as you go.

Once the room is mapped, use RViz's **2D Goal Pose** button and click anywhere in
it. The robot plans a route and drives there by itself, avoiding the pillars. If
a goal is ignored, it is almost always in a part of the room that is still
unmapped — drive closer and try again.

## All commands

| Command | What it does |
| --- | --- |
| `./run setup` | install the environment (first time, ~3 GB) |
| `./run build` | compile the robot packages |
| `./run sim` | Gazebo + RViz |
| `./run sim-headless` | no windows, useful for tests |
| `./run teleop` | drive from the keyboard |
| `./run slam` | build a map while driving |
| `./run nav2` | map + autonomous navigation |
| `./run stop` | shut everything down |
| `./run doctor` | check the setup and report problems |

## What the robot publishes

| Topic | Meaning |
| --- | --- |
| `/scan` | LiDAR, 720 beams over 360° |
| `/odom` | position and velocity estimate |
| `/tf`, `/tf_static` | coordinate frames |
| `/cmd_vel` | velocity commands (publish here to drive it) |
| `/clock` | simulation time |
| `/map` | occupancy grid, once SLAM is running |

Inspect them with `cd .. && pixi run -e classic bash`, then ordinary
`ros2 topic list`, `ros2 topic echo /scan`, and so on.

## Two things this simulation does not do

**The wheels do not spin, and walls do not stop the robot.** The plugin that
gives the robot its sideways motion drives the body directly instead of turning
the wheels, so wheel rotation is not simulated and a collision will not physically
halt the base. Navigation still avoids obstacles, because it plans using the map
built from the LiDAR.

**There is no camera.** macOS renders Gazebo camera images blank, so the RGB-D
camera and the AprilTag docking demo only work on Linux.

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
the whole reason `/scan`, SLAM and Nav2 work on a Mac at all.

## Contents

```
run                     the only command you need
launch/simulation.launch.py   Gazebo + RViz
launch/slam_nav2.launch.py    SLAM and Nav2
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
