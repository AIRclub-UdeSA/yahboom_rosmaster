# ROSMASTER X3 — Claude Code Project Hints

## Authoritative Source

After repo consolidation the single copy is:
`~/rosmaster_ws/src/yahboom_rosmaster/`

If you see two copies (also at `/home/juan/Documents/yahboom_rosmaster/`), the Documents one
is a duplicate and should be deleted or replaced with a symlink.

## Build

```bash
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select <package_name>
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

# Strafe right (holonomic test — only works in Fortress, not Classic)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"

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

### Why `gz:expressed_in` was removed from fdir1
`gz:expressed_in="base_link"` on `<fdir1>` is NOT supported in gz-physics 5.x (Fortress).
When present, DART applies fdir1 in the **local wheel-link frame**, which rotates with
the spinning wheel → rapidly changing friction direction → robot stays still (forces average
to zero). Without `gz:expressed_in`, DART applies fdir1 in the **world frame** (same
convention as ODE), which is correct at the robot's initial orientation (+x facing).

### Current friction model (mecanum_wheel.urdf.xacro)
```xml
<mu>1.0</mu>     <!-- high friction along fdir1 (roller axis) -->
<mu2>0.0</mu2>   <!-- zero friction perpendicular (roller rolls freely) -->
<fdir1>${fdir1}</fdir1>   <!-- world-frame, no gz:expressed_in -->
<slip1>0.0</slip1>
<slip2>1.0</slip2>        <!-- compliance in roller direction -->
```

### Limitation
fdir1 is world-frame (not body-frame). Strafing is correct when robot faces +x (initial
spawn orientation). After large rotations the effective roller axis drifts from the true
chassis-relative direction. For Nav2 holonomic navigation this is acceptable (controller
compensates). If precision post-rotation strafing is needed, investigate gz:expressed_in
support in gz-physics 6.x (Harmonic) or a Bullet physics backend.

## Known Issues — Status as of 2026-05-28

| Issue | Status | Where |
|-------|--------|--------|
| TF_OLD_DATA (RSP wall-clock poisoning) | **FIXED** | `TimerAction(period=2.0)` wraps RSP in fortress launch |
| TF_OLD_DATA (Fast DDS 50 Hz reordering) | **FIXED** | `update_rate: 30`, `publish_rate: 30.0` in `ros2_control.yaml` |
| Controller loading async race | **FIXED** | `--set-state active` (atomic) in fortress launch |
| Odometry circular-arc (wrong for holonomic) | **FIXED** | RK2 integration in `odometry.cpp` |
| Wheel spawn underground | **FIXED** | `-z 0.0325` in spawn args |
| Dual `/clock` publishers | **FIXED** | `/clock` removed from `ros_gz_bridge.yaml` |
| Wrong fdir1 vectors (unnormalized) | **FIXED** | `0.707107 ±0.707107 0` in `mecanum_wheel.urdf.xacro` |
| `gz:expressed_in` on fdir1 (wheel-frame rotation) | **FIXED** | Removed attribute; fdir1 now world-frame in DART |
| `mu2=0.0` contact instability | **FIXED** | `slip2=1.0` replaces mu2 for roller compliance |
| Double-robot spawn (ghost DDS RSP) | **FIXED** | `create -string` bypasses DDS topic subscription |
| Camera Classic plugin in Fortress | **FIXED** | Removed `libgazebo_ros_camera.so`; use native Fortress sensor |
| Sensors system plugin missing | **FIXED** | `gz-sim-sensors-system` + `gz-sim-imu-system` added to `empty.world` |
| RViz fixed frame wrong | **FIXED** | Changed to `odom` in `gazebo.rviz` |

## TODO — Verify on next session

**On the other machine, run these in order to confirm the branch is merge-ready:**

```bash
# 1. Build
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select \
  mecanum_drive_controller yahboom_rosmaster_description \
  yahboom_rosmaster_gazebo yahboom_rosmaster_bringup
source ~/rosmaster_ws/install/setup.bash

# 2. Launch
bash ~/rosmaster_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_gazebo.sh

# 3. After ~15s: verify one robot, two controllers
ros2 control list_controllers | sed 's/\x1b\[[0-9;]*m//g'
# Expected: joint_state_broadcaster[active], mecanum_drive_controller[active]

# 4. Test all three axes — each should produce actual Gazebo motion:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"   # strafe right
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"  # rotate

# 5. Restart test (double-spawn fix): Ctrl+C, wait 5s, relaunch immediately
#    → should still spawn ONE rosmaster_x3 in Entity Tree

# 6. If all pass: merge to main
git checkout main
git merge --ff-only fix/fortress-simulator
git push origin main
```

**Known observation from last session:** First strafe command after fresh launch may show
brief erratic motion before settling. If robot strafes cleanly after ~1s, the fix is working.
If it continues to spin/not strafe, open Claude Code and continue from this branch.
