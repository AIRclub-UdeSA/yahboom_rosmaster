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

## Known Remaining Issues (as of 2026-05-14)
- TF_OLD_DATA: partially fixed (spawn z-offset done, RSP delay still needs implementation)
- Controller loading: current bash script approach still has async race; needs `--set-state active`
- Odometry integration: uses circular-arc formula (wrong for holonomic); needs rotate-and-integrate
