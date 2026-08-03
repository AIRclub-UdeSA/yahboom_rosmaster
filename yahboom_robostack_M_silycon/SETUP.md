# Installing the ROSMASTER X3 simulation on a Mac

A complete walkthrough, from a Mac you have never used for robotics to a robot
driving around a simulated room, building a map, and navigating on its own.

No prior ROS experience is assumed. **You will not install ROS 2.** One tool,
`pixi`, downloads a self-contained copy of ROS 2 Humble and Gazebo into this
project folder. Nothing is installed system-wide, and deleting the folder
removes everything.

Budget **30–45 minutes**, most of it waiting on one download.

---

## What you are actually installing, and why

ROS 2 has no official macOS build. RoboStack solves this by repackaging ROS 2
as **conda** packages, which do work on macOS. `pixi` is the tool that reads
this project's `pixi.toml`, resolves those packages, and drops them into a
`.pixi/` folder inside the repository.

The practical consequence: there is no `source /opt/ros/humble/setup.bash` step
and no `apt install`. Every command goes through `./run`, which enters the
environment for you.

One more thing to know up front. This project simulates the robot in **Gazebo
Classic**, not the Gazebo Fortress the rest of the repository targets. Fortress
starts its renderer on a background thread, and macOS only permits window
creation on the main thread, so on a Mac Fortress cannot open its GUI and cannot
run any sensor that needs rendering — the LiDAR would crash the simulator.
Gazebo Classic opens its window normally and its LiDAR works by asking the
physics engine where the walls are instead of drawing them. That is the entire
reason `/scan`, SLAM and Nav2 work on a Mac at all.

---

## Before you start

| Requirement | Why |
| --- | --- |
| **Apple Silicon Mac** (M1/M2/M3/M4) | The environment is built for `osx-arm64`. Intel Macs are not supported. |
| **macOS 12 or newer** | Older versions have not been tested. |
| **~6 GB free disk** | The environment alone is 4.3 GB once installed. |
| **A decent internet connection** | The first setup downloads several GB. |

Check your chip if you are unsure — this must print `arm64`:

```bash
uname -m
```

---

## Step 1 — Xcode Command Line Tools

This provides `git` and the compilers the build needs.

```bash
xcode-select --install
```

A dialog appears; accept it and wait. If it is already installed you will see
`command line tools are already installed`, which is fine. Verify:

```bash
git --version
```

## Step 2 — Homebrew

Skip if you already have `brew`. Otherwise:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Homebrew prints two `export` lines at the end telling you to add itself to your
PATH. **Run them.** If you skip that, the `pixi` install in the next step will
appear to succeed but the command will not be found.

Verify:

```bash
brew --version
```

## Step 3 — pixi

```bash
brew install pixi
```

Verify:

```bash
pixi --version        # e.g. pixi 0.74.0
```

<details>
<summary>No Homebrew? Install pixi directly</summary>

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Then restart your terminal so `~/.pixi/bin` lands on your PATH.
</details>

## Step 4 — Get the code

> **Which repository?** The macOS support currently lives on the fork below.
> The club repository `AIRclub-UdeSA/yahboom_rosmaster` does **not** have the
> `yahboom_robostack_M_silycon/` folder yet — it arrives there when pull
> request #2 is merged. Until then, clone the fork.

```bash
mkdir -p ~/Gits && cd ~/Gits
git clone https://github.com/bchax/yahboom_rosmaster.git
cd yahboom_rosmaster/yahboom_robostack_M_silycon
```

Everything from here runs **from inside this folder**.

Confirm you are in the right place — this should list `run`, `launch`, `worlds`,
`rviz`, `scripts`:

```bash
ls
```

## Step 5 — Install the environment

```bash
./run setup
```

This is the long one. It downloads ROS 2 Humble, Gazebo Classic, RViz, Nav2 and
slam_toolbox — **4.3 GB on disk** when finished. Ten to thirty minutes is
normal on a first run.

It is safe to re-run if it is interrupted; it resumes rather than restarting.

Because `pixi.lock` is committed to the repository, every student gets byte-for-byte
the same package versions. If it works for one of you, it works for all of you.

## Step 6 — Build the robot packages

```bash
./run build
```

Fast — a few seconds. You will see five packages finish:

```
Finished <<< yahboom_rosmaster_description
Finished <<< yahboom_rosmaster_docking
Finished <<< yahboom_rosmaster_gazebo
Finished <<< yahboom_rosmaster_navigation
Finished <<< yahboom_rosmaster_msgs
Summary: 5 packages finished
```

You may see warnings about `install_name_tool` and *"code signature"* from
`yahboom_rosmaster_msgs`. **These are normal on macOS** and can be ignored.

## Step 7 — Check everything before launching

```bash
./run doctor
```

Every line should read `ok`, ending with:

```
All good. Start with:  ./run sim
```

If anything says `FAIL`, it tells you which step to repeat. Run this first
whenever something misbehaves later.

## Step 8 — Launch

```bash
./run sim
```

**Two windows open.** Gazebo shows the robot in a 6 × 6 m walled room with three
pillars. RViz shows what the robot itself perceives — the robot model, its
coordinate frames, and a ring of red LiDAR points tracing the walls.

Leave this terminal running. Closing it stops the simulation.

> Gazebo can take 10–20 seconds to appear the first time, while macOS builds its
> dynamic-library cache. Later launches are quicker.

## Step 9 — Drive it

Open a **second terminal**, and go to the same folder:

```bash
cd ~/Gits/yahboom_rosmaster/yahboom_robostack_M_silycon
./run teleop
```

| Key | Action |
| --- | --- |
| `i` | forward |
| `,` | backward |
| `j` / `l` | turn left / right |
| `k` | stop |
| `b` | holonomic mode — then **Shift** + movement keys strafes sideways |

The strafing is the interesting part: this is a **mecanum** robot, so it can
slide sideways without turning. Ordinary differential-drive robots cannot.

**This terminal must stay focused for the keys to register.**

## Step 10 — Mapping and navigation

With the simulation still running, in that second terminal (press `Ctrl+C` to
stop teleop first):

```bash
./run slam        # builds a map as you drive
```

or

```bash
./run nav2        # builds a map AND navigates on its own
```

Then open a **third terminal** for `./run teleop` and drive around. Watch the
map fill in inside RViz.

**Drive the whole room before setting a goal.** The robot can only plan through
space its LiDAR has already seen. Once the room is mapped, click RViz's
**2D Goal Pose** button and then click somewhere in the map — the robot plans a
route and drives there by itself, avoiding the pillars.

If a goal is ignored or the robot reports success without moving, the goal was
almost certainly in unmapped space. Drive closer and try again.

## Step 11 — Shut down

```bash
./run stop
```

Always use this rather than just closing windows. ROS on macOS does not reliably
clean up its own processes, and a leftover node will break the *next* launch in
ways that look like a network fault.

---

## Knowing it actually works

Open another terminal and look at the live data:

```bash
cd ~/Gits/yahboom_rosmaster
pixi run -e classic bash        # you are now inside the ROS environment
ros2 topic list
```

You should see `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, `/joint_states`.

```bash
ros2 topic echo /scan --once
```

Sanity checks on a robot sitting at its spawn point in the 6 × 6 m room:

| Field | Expected |
| --- | --- |
| `ranges` length | **1080** beams |
| `range_min` / `range_max` | **0.25** / **12.0** m |
| Distance to the walls | roughly **2.9–3.0 m** in every direction |

If every range is under 0.25 m, the LiDAR is seeing the robot's own body — that
is a known failure mode and means something is wrong with the model.

Type `exit` to leave the environment.

---

## Command reference

| Command | What it does |
| --- | --- |
| `./run setup` | install the environment (first time only, 4.3 GB) |
| `./run build` | compile the robot packages |
| `./run sim` | Gazebo + RViz |
| `./run sim-headless` | no windows — useful for testing |
| `./run teleop` | drive from the keyboard |
| `./run slam` | build a map while driving |
| `./run nav2` | map + autonomous navigation |
| `./run stop` | shut everything down |
| `./run doctor` | check the setup and report problems |

---

## Troubleshooting

**Always run `./run doctor` first.** It checks the platform, pixi, the
environment, the build, the assets and the Gazebo plugins, and names the step to
repeat.

| Symptom | Cause and fix |
| --- | --- |
| `pixi is not installed` | Step 3 did not finish, or Homebrew is not on your PATH. Re-run the `export` lines Homebrew printed, open a new terminal, try again. |
| A window never appears, or a command hangs | A previous run left processes behind. `./run stop`, then retry. |
| Nothing publishes, nodes cannot see each other | A Mac exposes ~20 network interfaces and ROS can pick different ones for different programs. This folder pins everything to loopback automatically. If you changed `ROS_LOCALHOST_ONLY`, set it back to `1`. |
| `./run build` fails | Make sure `./run setup` finished. Then `cd .. && pixi run clean-classic` and build again. |
| Robot drifts when you drive straight | **This is deliberate** — see below. |
| Very slow, or fans spinning up | Normal during `./run setup`. If it persists during simulation, close the Gazebo window and use `./run sim-headless`. |

---

## Two things that surprise people

**The robot drifts on purpose.** Drive straight and it curves gently to one
side. A real mecanum base is never perfectly calibrated, so a "go forward"
command also produces a little sideways and rotational motion. The simulation
copies that, with the amount drawn at random each launch, so you cannot tune
your code to one fixed error. This is exactly what closed-loop control is for —
SLAM and Nav2 correct for it continuously. To switch it off while debugging
something else:

```bash
cd .. && pixi run -e classic ros2 launch \
  yahboom_robostack_M_silycon/launch/simulation.launch.py motion_bias:=false
```

**The wheels do not spin, and walls do not stop the robot.** The plugin that
gives the robot its sideways motion drives the body directly rather than turning
the wheels, so wheel rotation is not simulated and a collision will not
physically halt the base. Navigation still avoids obstacles, because it plans
from the map the LiDAR built.

There is also **no camera** — macOS renders Gazebo camera images blank, so the
RGB-D camera and the AprilTag docking demo only work on Linux. If you need exact
mecanum wheel physics or the camera, use the Gazebo Fortress backend on Linux
(`pixi run sim` from the repository root).

---

## Uninstalling

Everything lives inside the cloned folder. To reclaim the disk:

```bash
cd ~/Gits/yahboom_rosmaster
pixi run clean-classic     # remove build output only
rm -rf .pixi               # remove the 4.3 GB environment
```

Or simply delete `~/Gits/yahboom_rosmaster`. Nothing was installed
system-wide except Homebrew and pixi themselves.

---

## Where to go next

- `README.md` in this folder — day-to-day usage and what each topic means.
- `../README.md` — the Linux / Gazebo Fortress reference simulation. That is the
  backend this one stands in for, and it documents the full sensor set,
  the odometry topics and the motion profiles.
- `launch/simulation.launch.py` — how Gazebo, the robot model and RViz are
  actually wired together. A good first file to read when you want to change
  something.
