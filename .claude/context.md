# ROSMASTER X3 — Simulation Debugging History

This document records issues encountered during Gazebo Fortress bring-up, how each was solved,
and lessons learned. Update it when new issues are solved.

---

## Issue 1: Gazebo Classic strafing never worked

**Symptom:** Publishing `cmd_vel` with `linear.y != 0` produced no lateral motion in Classic.

**Root cause:** Gazebo Classic's ODE physics engine ignores the `fdir1` friction direction
attribute for cylinder collisions. The holonomic constraint cannot be enforced.

**Fix:** Switched to Gazebo Fortress (Ignition) which uses the DART physics engine and
respects `gz:expressed_in="base_link"` on fdir1 vectors, locking the roller axis direction
to the chassis frame even as the wheel spins. Classic launch files were deleted.

**Lesson:** Never use Classic for mecanum simulation. Fortress is required.

---

## Issue 2: Wrong fdir1 vectors — strafing still broken in Fortress

**Symptom:** Robot moved diagonally or not at all when a pure strafe command was sent.

**Root cause:** The `fdir1` values in `rosmaster_x3.urdf.xacro` were non-normalized and the
diagonal pairings were wrong.

**Correct mecanum kinematics:**
- Front-left and back-right share one roller axis direction: `fdir1="0.707107 -0.707107 0"`
- Front-right and back-left share the other: `fdir1="0.707107 0.707107 0"`

**File fixed:** `yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro`

---

## Issue 3: Sphere wheel collision penetrating ground — violent jitter at spawn

**Symptom:** Robot shook violently immediately after spawning. Wheels appeared to vibrate.

**Root cause:** Sphere collision radius = 0.0325 m. Spawn at z=0 placed the wheel center at
z=0, putting the bottom of the sphere at z=-0.0325 m (underground). With kp=200000 N/m, the
contact generated ~6500 N on a 1.7 kg robot — 390g of upward force per wheel.

**Fix:** Add `-z 0.0325` to the spawn node arguments in `rosmaster_gazebo_fortress.launch.py`.
This places the wheel center at the correct height so the sphere just touches the ground plane.

**File fixed:** `yahboom_rosmaster_gazebo/launch/rosmaster_gazebo_fortress.launch.py`

---

## Issue 4: Contact parameters causing oscillations / unrealistic behavior

**Symptom:** Wheels bounced, robot oscillated vertically, contact was unstable.

**Root cause:** kp=1000000 (too stiff for 50g wheels), kd=100 (too soft, kd/kp=0.0001),
mu2=0.0 (exactly zero causes numerical instability in DART's contact solver).

**Fix (in mecanum_wheel.urdf.xacro):**
- `kp` → 200000 (softer contact, appropriate for small wheels)
- `kd` → 2000 (kd/kp ≈ 0.01, prevents bouncing)
- `mu2` → 0.1 (not zero; rollers are nearly frictionless transversely but not degenerate)

---

## Issue 5: Dual /clock publisher causing RViz "jump back in time" loops

**Symptom:** RViz printed "Detected jump back in time" warnings continuously. Displays reset
every few seconds.

**Root cause:** Both the Gazebo internal ROS bridge AND `ros_gz_bridge`'s `parameter_bridge`
were publishing to `/clock`. This created two publishers with slightly different timestamps,
causing `rclcpp` time source to detect spurious backward jumps.

**Fix:** Remove the clock entry from `ros_gz_bridge.yaml`. The Gazebo internal bridge
publishes `/clock` automatically — no explicit bridging needed.

**File fixed:** `yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml`

---

## Issue 6: RViz "odom frame not found"

**Symptom:** RViz showed "Fixed Frame [odom] does not exist" at startup.

**Root cause:** RViz's fixed frame was set to `odom`. The mecanum_drive_controller only
publishes the `odom` → `base_footprint` TF transform after it becomes active (at t≈14s).
RViz had no frame to anchor to during the first ~14 seconds.

**Fix:** Change the RViz fixed frame to `base_footprint`. This frame is always published by
`robot_state_publisher` directly from the URDF, regardless of controller state.

**File fixed:** `yahboom_rosmaster_gazebo/rviz/gazebo.rviz`

---

## Issue 7: TF_OLD_DATA — perpetual transform errors after startup

**Symptom:** `[tf2]: Lookup would require extrapolation into the past. Requested time X
but the earliest data is at time Y` printed continuously. Robot model in RViz displayed
incorrectly or at wrong position.

**Root cause (confirmed):** `robot_state_publisher` starts at t=0 with `use_sim_time=true`.
At that moment, no `/clock` topic exists yet (Gazebo hasn't started publishing). RCL falls
back to the system wall clock (~1.778×10⁹ seconds since epoch). RSP publishes a TF transform
with this wall-clock timestamp. TF2 stores this as the "latest known transform time."

When Gazebo later starts publishing `/clock` (sim-time ~37s, robot time ~1179s), all new
transforms appear to be from the distant past relative to the wall-clock-poisoned buffer.
TF2's lookup always fails with TF_OLD_DATA.

**Fix:** Wrap `robot_state_publisher` in `TimerAction(period=2.0, ...)` in the launch file.
Gazebo publishes its first `/clock` message within ~1s of starting. The 2s delay ensures
RCL has switched to sim-time mode before RSP publishes any transforms.

**Status:** Fix identified, not yet implemented as of 2026-05-14.

**Wrong fix attempted:** Changed `transform.header.stamp = time` to `get_node()->get_clock()->now()`
in `mecanum_drive_controller.cpp`. This was incorrect — `get_clock()->now()` can also return
wall-clock time during the window before sim-time is established. Always use the `time`
parameter passed to `update()` by the controller_manager. This must be reverted.

---

## Issue 8: Controllers staying in UNCONFIGURED state after load

**Symptom:** `ros2 control list_controllers` showed controllers in `unconfigured` state even
after the activation script ran. `/cmd_vel` had no effect.

**Root cause:** `gz_ros2_control/GazeboSimSystem` creates `controller_manager` but does NOT
auto-load controllers from YAML. The activation bash script used a 3-step sequence:
1. `ros2 control load_controller <name>` → creates controller in UNCONFIGURED state
2. `ros2 control set_controller_state <name> inactive` → supposed to trigger `on_configure()`
3. `ros2 control set_controller_state <name> active` → activate

Step 2 fails silently due to an asynchronous race in Humble's lifecycle callbacks. The
controller stays unconfigured, and step 3 then fails because configure was never called.

**Also:** The bash script grepped `ros2 control list_controllers` output to check state.
The output uses ANSI color codes (`\e[92m` etc.) so `grep "^joint_state_broadcaster"` never
matched — the script silently did nothing and reported empty state for all controllers.

**Correct approach:** Use `ros2 control load_controller --set-state active <name>` which
atomically handles load→configure→activate in one call. This is already demonstrated in
`yahboom_rosmaster_bringup/launch/load_ros2_controllers.launch.py`.

**Status:** Fix identified, not yet implemented as of 2026-05-14.

---

## Issue 9: Double sentinel bug in controller on_configure()

**Symptom:** Velocity command lag was twice the configured `cmd_vel_timeout`. Deceleration
ramp behaved incorrectly.

**Root cause:** `on_configure()` called `reset()` (which itself pushes 2 zero-velocity
sentinels into `previous_commands_`), then also pushed 2 more sentinels directly. The queue
had 4 entries instead of 2, doubling the effective command delay.

**Fix:** Remove the duplicate sentinel-push code from `on_configure()` — rely only on the
sentinels added by `reset()`.

**File fixed:** `mecanum_drive_controller/src/mecanum_drive_controller.cpp`

---

## Issue 10: Ghost DDS nodes after Ctrl+C

**Symptom:** After killing the simulation, `ros2 node list` still shows multiple instances
of `/controller_manager` and other nodes for several minutes.

**Root cause:** ROS 2's DDS discovery layer (FastDDS) caches participant information and
doesn't immediately clean up when processes die. These are phantom endpoints.

**Workaround:** Use `pgrep` to check actual process existence before launching:
```bash
pgrep -a "gz sim\|controller_manager\|robot_state_pub" 
```
Ghost nodes expire after ~5 minutes when the DDS lease duration times out.
Do not try to kill them — there is no process to kill.

---

## Good Practices for This Codebase

1. **Build incrementally:** `colcon build --symlink-install --packages-select <pkg>` — never
   rebuild the entire workspace unless you changed a dependency.

2. **Always source after build:**
   ```bash
   source ~/rosmaster_ws/install/setup.bash
   ```

3. **Check controller state without ANSI issues:**
   ```bash
   ros2 control list_controllers | sed 's/\x1b\[[0-9;]*m//g'
   ```

4. **TF debugging:**
   ```bash
   ros2 run tf2_tools view_frames  # generates frames.pdf
   ros2 run tf2_ros tf2_monitor    # live TF latency stats
   ```

5. **Quick movement test after launch:**
   ```bash
   # Strafe — confirms holonomic physics working
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"
   ```

6. **Use --set-state active** for controller loading, never the 3-step sequence.

7. **Use `time` parameter** for all TF stamps in controller update(), never `get_clock()->now()`.

8. **Delay RSP** to after Gazebo clock is available (2s after Gazebo server starts).
