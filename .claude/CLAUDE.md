# ROSMASTER X3 — Claude Code Project Hints

## Authoritative Source

After repo consolidation the single copy is:
`~/rosmaster_ws/src/yahboom_rosmaster/`

If you see two copies (also at `/home/juan/Documents/yahboom_rosmaster/`), the Documents one
is a duplicate and should be deleted or replaced with a symlink.

## Build

```bash
cd ~/rosmaster_ws
colcon build --symlink-install --allow-overriding mecanum_drive_controller --packages-select <package_name>
```

After editing C++ source files you may need to touch them to force CMake rebuild detection:
```bash
touch ~/rosmaster_ws/src/yahboom_rosmaster/<pkg>/src/*.cpp
```

## Key Packages

| Package | Purpose |
|---------|---------|
| `yahboom_rosmaster_gazebo` | Fortress sim launch, bridge config, ros2_control.yaml, worlds |
| `yahboom_rosmaster_description` | URDF/xacro, robot_state_publisher launch |
| `mecanum_drive_controller` | Custom mecanum odometry + velocity controller |
| `yahboom_rosmaster_bringup` | Navigation + real-robot launch files |
| `yahboom_rosmaster_docking` | AprilTag dock pose publisher |

## Critical Rules (learned from debugging)

### TF Stamps
**Always use the `time` parameter from `update()` for TF stamps, never `get_clock()->now()`:**
```cpp
// CORRECT
transform.header.stamp = time;

// WRONG — may return wall-clock time before sim-time stabilizes, poisoning TF2 buffer
transform.header.stamp = get_node()->get_clock()->now();
```

### Controller Loading
**Use `--set-state active` for atomic load→configure→activate. Never use the 3-step sequence:**
```bash
# CORRECT (atomic)
ros2 control load_controller --set-state active joint_state_broadcaster

# WRONG — set_controller_state inactive/active fails due to async race in Humble
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster inactive
ros2 control set_controller_state joint_state_broadcaster active
```

### ANSI Escape Codes
`ros2 control list_controllers` output contains ANSI color codes. Never grep it raw:
```bash
# CORRECT
ros2 control list_controllers | sed 's/\x1b\[[0-9;]*m//g' | grep "active"

# WRONG — grep "^controller_name" never matches because lines start with \e[92m
ros2 control list_controllers | grep "^joint_state_broadcaster"
```

### robot_state_publisher Startup
RSP must start AFTER Gazebo is publishing `/clock`. A 2-second delay is sufficient.
If RSP starts at t=0 with `use_sim_time=true` and no clock exists, it falls back to
wall-clock (~1.778×10⁹ s) and permanently poisons the TF2 buffer with that timestamp.
All sim-time TF subsequently appears ancient → perpetual `TF_OLD_DATA` errors.

### Ghost DDS Nodes
After killing the simulation (`Ctrl+C`), `ros2 node list` shows zombie nodes for ~5 minutes.
Use `pgrep -a ruby\|gz\|gazebo\|controller_manager` to check real process state.
Don't try to kill ghosts — they expire on their own.

## Controller Config Location
`yahboom_rosmaster_gazebo/config/ros2_control.yaml`

This file is loaded by the `gz_ros2_control/GazeboSimROS2ControlPlugin` via the
`<parameters>` tag in the URDF. It sets wheel names, wheel_separation, wheel_base,
publish_rate, etc. for the mecanum_drive_controller.

## Simulation Launch
```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py
```

Expected startup sequence:
- t=0s: Gazebo server + client start
- t=2s: robot_state_publisher starts (after clock is available)
- t=3s: robot spawned in Gazebo
- t=5s: ros_gz_bridge + image_bridge start
- t=10s: twist_to_stamped.py converter starts
- t=12s: controllers load+activate
- t=5s: RViz starts (delayed by OpaqueFunction)

Verify controllers are active:
```bash
ros2 control list_controllers | sed 's/\x1b\[[0-9;]*m//g'
# Expected: joint_state_broadcaster[active], mecanum_drive_controller[active]
```

## Test Movement
```bash
# Forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Strafe right (holonomic) — MUST use /cmd_vel (not direct controller topic)
# twist_to_stamped adds the correct sim-time stamp; direct TwistStamped with
# no stamp gets rejected by the 0.5s cmd_vel_timeout in the controller.
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"

# Rotate
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
```

## TF Diagnostics

When TF_OLD_DATA appears, run these to identify the source:
```bash
# How many publishers on /joint_states? (should be exactly 1)
ros2 topic info /joint_states --verbose

# How many publishers on /tf? (should be robot_state_publisher + mecanum controller)
ros2 topic info /tf --verbose

# Live TF tree — check odom→base_footprint→base_link→wheel_links are all connected
ros2 run tf2_tools view_frames

# Monitor a specific frame for TF_OLD_DATA source
ros2 run tf2_ros tf2_monitor front_left_wheel_link

# Check /clock publisher (should be exactly 1: Gazebo's internal bridge)
ros2 topic info /clock --verbose
```

If more than one publisher appears on `/joint_states` or `/tf` for wheel frames,
that second publisher is the root cause of TF_OLD_DATA (conflicting timestamps).

## Mecanum Wheel Physics Notes (DART / Fortress)

### How `gz:expressed_in` works in gz-physics 5.x
`ignition:expressed_in="base_link"` on `<fdir1>` locks the friction direction to the **robot chassis
frame** (base_link), not to the spinning wheel-link frame.

**CRITICAL NAMESPACE BUG (confirmed 2026-05-29):** gz-physics 5.3.2's DART plugin parses the
attribute as a raw string literal `"ignition:expressed_in"`. It does **NOT** recognise
`"gz:expressed_in"`. Using `gz:expressed_in` causes the attribute to be silently ignored —
fdir1 then defaults to world-frame interpretation, which causes rotation+forward drift instead
of strafing. Confirmed by inspecting the plugin binary:
```
strings libignition-physics5-dartsim-plugin.so.5.3.2 | grep expressed_in
→ ignition:expressed_in   (only this string; "gz:expressed_in" is absent)
```

**This is the required config for holonomic strafing.** Without `ignition:expressed_in`, fdir1 is
interpreted in the **world frame**, which only works at the robot's initial orientation and
breaks after any rotation.

### Current friction model (mecanum_wheel.urdf.xacro)
```xml
<mu>1.0</mu>                                      <!-- high friction along fdir1 (roller axis) -->
<mu2>0.0</mu2>                                    <!-- zero friction perpendicular (roller rolls freely) -->
<fdir1 ignition:expressed_in="base_link">${fdir1}</fdir1>   <!-- chassis-frame, rotates with robot body -->
<!-- xmlns:ignition="http://ignitionrobotics.org/schema" MUST be declared in <robot> element -->
<!-- gz-physics 5.x (Fortress): use ignition:expressed_in (NOT gz:expressed_in) -->
```

### twist_to_stamped clock rule
`twist_to_stamped.py` must run with **wall clock** (NOT `use_sim_time:=true`).

If `use_sim_time:=true` is passed to twist_to_stamped via `ExecuteProcess`, rclpy's
ROS clock fails to initialize in the subprocess context and `get_clock().now()` returns
epoch-zero (`sec=0, nanosec=0`). The controller then computes:

    age = sim_time - stamp(0) = 54s >> 0.5s timeout → brakes every cycle → zero motion

With `use_sim_time=false` (default), twist_to_stamped stamps with wall time (~1.78×10⁹ s):

    age = sim_time(54s) - wall_time(1.78e9s) = large negative → no timeout → wheels spin ✓

### Correct test command for strafing
Always test strafing via `/cmd_vel` (Twist), **not** directly to the controller topic:
```bash
# CORRECT — goes through twist_to_stamped which adds wall-clock stamp (negative age, no timeout)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"

# WRONG — zero header.stamp → controller sees age=sim_time >> 0.5s timeout → brakes
ros2 topic pub --rate 10 /mecanum_drive_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {y: 0.3}}}"
```

## Known Issues — Status as of 2026-05-29

| Issue | Status | Where |
|-------|--------|--------|
| TF_OLD_DATA (RSP wall-clock poisoning) | **FIXED** | `TimerAction(period=2.0)` wraps RSP in fortress launch |
| TF_OLD_DATA (Fast DDS 50 Hz reordering) | **FIXED** | `update_rate: 30`, `publish_rate: 30.0` in `ros2_control.yaml` |
| Controller loading async race | **FIXED** | `--set-state active` (atomic) in fortress launch |
| Odometry circular-arc (wrong for holonomic) | **FIXED** | RK2 integration in `odometry.cpp` |
| Wheel spawn underground | **FIXED** | `-z 0.0325` in spawn args |
| Dual `/clock` publishers | **FIXED** | `/clock` removed from `ros_gz_bridge.yaml` |
| Wrong fdir1 vectors (unnormalized) | **FIXED** | `0.707107 ±0.707107 0` in `mecanum_wheel.urdf.xacro` |
| `gz:expressed_in` silently ignored by DART plugin | **FIXED** | Use `ignition:expressed_in="base_footprint"` — gz-physics 5.3.2 only parses `ignition:expressed_in`, not `gz:expressed_in`; confirmed in generated SDF |
| Strafing broken (DART/gz_ros2_control interaction) | **OPEN** | fdir1 + sphere gives correct forward/rotation but strafing produces -y for +y command + ~50° rotation. Official Gazebo mecanum demo (MecanumDrive plugin) DOES strafe correctly with same fdir1. Root cause: unknown DART interaction with gz_ros2_control velocity constraints. See Strafing Physics Investigation section. |
| Passive rollers don't strafe (DART ignores passive joints in LCP) | **WONT-FIX** | DART contact solver ignores passive joint DOF; isotropic contact forces cancel during strafe |
| Bullet physics doesn't support anisotropic friction | **WONT-FIX** | Bullet plugin ignores mu2/fdir1; only isotropic friction available |
| Zero-stamp cmd_vel (twist_to_stamped clock bug) | **FIXED** | Removed `use_sim_time:=true` from twist_to_stamped; rclpy ROS clock fails in subprocess → stamp=0 → timeout. Wall clock gives negative age → no brake |
| Double-robot spawn (ghost DDS RSP) | **FIXED** | `create -string` bypasses DDS topic subscription |
| Camera Classic plugin in Fortress | **FIXED** | Removed `libgazebo_ros_camera.so`; use native Fortress sensor |
| Sensors system plugin missing | **FIXED** | `gz-sim-sensors-system` + `gz-sim-imu-system` added to `empty.world` |
| RViz fixed frame wrong | **FIXED** | Changed to `odom` in `gazebo.rviz` |
| `pal_statistics_msgs` phantom dependency | **FIXED** | Removed from `mecanum_drive_controller/package.xml` + `CMakeLists.txt` (never used in source) |
| `gazebo_ros` (Classic) in Fortress build | **FIXED** | Removed from `yahboom_rosmaster_gazebo/CMakeLists.txt` (pure launch pkg, no C++ deps needed) |

## DDS Ghost Publisher Killer — REQUIRED before every sim launch

Zombie robot_state_publisher and ign gazebo processes from previous sessions publish stale
TRANSIENT_LOCAL robot_description messages. gz_ros2_control reads one of these at spawn time
and initializes hardware without command_interfaces → controllers fail to load.

```bash
pkill -9 -f "ign|ros2 launch|robot_state_publisher|controller_manager|ros_gz|twist_to_stamped|image_bridge|parameter_bridge" 2>/dev/null
sleep 3
kill -9 $(pgrep -f robot_state_publisher) 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
source /opt/ros/humble/setup.bash && ros2 daemon stop 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# Verify: should say "no publishers (clean)"
ros2 topic info /robot_description 2>/dev/null | grep "Publisher count" || echo "clean"
```

## Strafing Physics Investigation (2026-05-31) — DART Limitations

### Approaches tried and results

**Approach A — Passive roller cylinders (ABANDONED)**
Added 4 cylinder rollers per wheel as passive revolute joints. Theory: DART's LCP solver
would use the passive DOF to produce anisotropic friction (roller spins freely = zero friction
in that direction).
**Result:** DART's contact LCP does NOT use passive joint DOF for friction anisotropy. The
contact solver treats friction as isotropic at the contact point, ignoring that the roller
joint is free. All 4 wheels produce zero net lateral force → no strafing. 8 rollers also
hangs DART (32 passive joints too slow for constraint solver at 1ms timestep).

**Approach B — Bullet physics + fdir1 (ABANDONED)**
Switched to `ignition-physics5-bullet-plugin.so` (available, installed).
**Result:** Bullet physics ignores ODE-style friction parameters entirely (mu, mu2, fdir1
are not supported). Uses isotropic friction only. No strafing.

**Approach C — Sphere collision + fdir1 + ignition:expressed_in (CURRENT STATE)**
Sphere collision (single contact point per wheel) with:
- `mu=1.0` along fdir1 (grip axis), `mu2=0.0` perpendicular (free rolling)
- `ignition:expressed_in="base_footprint"` → confirmed in generated SDF
- FL/BR: fdir1="0.707107 -0.707107 0", FR/BL: fdir1="0.707107 0.707107 0"
- Matches official Gazebo mecanum demo fdir1 assignments exactly

**Test results (2026-05-31):**
- Forward: Δx=+1.49m in 5s at 0.3 m/s (99%) ✓
- Rotation: CCW for angular.z=+0.5, correct direction ✓
- Strafe: Δy=-0.026m + Δyaw=-48.8° (wrong direction AND significant rotation) ✗

### Why strafing fails with Approach C

The official Gazebo mecanum demo (native SDF + MecanumDrive plugin + DART) WORKS for strafing.
Our robot (URDF + gz_ros2_control + DART) does NOT. The fdir1 values and expressed_in are
identical. The key difference is the motion control system.

**MecanumDrive plugin**: Ignition native plugin that computes wheel velocities from body cmd.
Applies JointVelocityCommandComponent directly in the physics system thread.

**gz_ros2_control + mecanum_drive_controller**: Also applies JointVelocityCommandComponent.
Mechanically equivalent to MecanumDrive.

Root cause is not yet identified. Empirical behavior: any fdir1 assignment produces strafing
in -y direction for +y command + ~50° unwanted yaw rotation. Torque analysis shows torques
should cancel (zero net yaw) but empirically they don't. DART may have a constraint solver
interaction with gz_ros2_control velocity commands that doesn't occur with MecanumDrive.

**Note:** The strafing failure is NOT due to the expressed_in one-shot bug (that would only
affect headings after rotation). The robot fails to strafe even at initial yaw=0.

### cmd_vel_timeout workaround (required for testing)

The wall-clock stamp from `twist_to_stamped.py` makes `cmd_vel_timeout` never fire
(age = sim_time - wall_time ≈ -1.78e9 s << 0 → no timeout → last command runs forever).
**Always send a zero cmd_vel after each test:**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## TODO — Next session

**Verified working (2026-05-31):** Forward Δx=1.49m ✓, Rotation CCW ✓
**Strafe: BROKEN** with all physics-based approaches tried. Options:

1. **Add MecanumDrive plugin** alongside mecanum_drive_controller: official demo proves
   this combination works for strafing. MecanumDrive handles wheel kinematics (not body
   velocity directly), so ground friction still matters. May satisfy "physics-based" requirement
   since it's not a body velocity override.

2. **Accept limitation**: forward + rotation work correctly. Strafing doesn't. Document as
   DART/gz_ros2_control limitation.

3. **Upgrade Gazebo**: Fortress (gz-physics 5.3.2) is EOL. Newer versions may fix the
   expressed_in one-shot bug AND the URDF fdir1 interaction issue.

**Remaining tests (blocked on strafe):**
- Restart test (step 5): Ctrl+C, wait 5s, relaunch → verify one robot
- Merge to main (step 6) when user is satisfied

**Session history:** `fix/fortress-simulator` branch. Sphere + fdir1 + ignition:expressed_in
approach is in place. VelocityControl and passive roller approaches both reverted.

```bash
# 0. Install ROS 2 Humble + Gazebo Fortress (new machine only — skip if already installed)
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create workspace if not already present, then install rosdep deps
mkdir -p ~/rosmaster_ws/src
# (repo should already be cloned; if not: cd ~/rosmaster_ws/src && git clone https://github.com/juan-kaplan/yahboom_rosmaster.git)
cd ~/rosmaster_ws
rosdep init || true   # safe to ignore "already initialized" error
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 1. Build
cd ~/rosmaster_ws
colcon build --symlink-install --allow-overriding mecanum_drive_controller \
  --packages-select mecanum_drive_controller yahboom_rosmaster_description \
  yahboom_rosmaster_gazebo yahboom_rosmaster_bringup
source ~/rosmaster_ws/install/setup.bash

# 2. Launch
bash ~/rosmaster_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_gazebo.sh

# 3. After ~15s: verify one robot, two controllers
ros2 control list_controllers | sed 's/\x1b\[[0-9;]*m//g'
# Expected: joint_state_broadcaster[active], mecanum_drive_controller[active]

# 4. Test all three axes — each should produce actual Gazebo motion:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"    # forward
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}" # strafe (needs --rate, --once times out in 0.5s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"   # rotate

# 5. Restart test (double-spawn fix): Ctrl+C, wait 5s, relaunch immediately
#    → should still spawn ONE rosmaster_x3 in Entity Tree

# 6. If all pass: merge to main
git checkout main
git merge --ff-only fix/fortress-simulator
git push origin main
```

**What changed (2026-05-29):** Root cause identified: gz-physics 5.3.2 DART plugin computes
`ignition:expressed_in` frame transform ONCE at model load, not per physics step. After any
robot rotation, the friction direction drifts → positive feedback → rotation+forward instead of strafe.

Fix: bypassed DART anisotropic friction entirely. Added `gz-sim-velocity-control-system` plugin
to URDF (topic `/cmd_vel_world`). Added `mecanum_velocity_control.py` that transforms body-frame
`/cmd_vel` → world-frame using odom heading, publishes to `/cmd_vel_world_frame`, bridged to
Ignition via ros_gz_bridge. mecanum_drive_controller still handles wheel IK + odometry.
