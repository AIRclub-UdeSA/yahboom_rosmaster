# Fortress simulator branch analysis

This document reviews the `fix/fortress-simulator` branch from `main` through
the final working mecanum simulation. It records the commit history, the
important errors discovered along the way, the approaches that were rejected,
and the final goal that the branch now satisfies.

## Final goal

The final goal was to make the ROSMASTER X3 simulation work in ROS 2 Humble with
Gazebo Fortress while preserving physics-based mecanum behavior:

- The robot must move through wheel-ground contact forces, not by teleporting,
  setting model pose, or overriding body velocity.
- The public command API remains `/cmd_vel`.
- Wheel slip should remain visible as odometry error, so odometry must not come
  from Gazebo ground truth.
- The simulator should expose a clean ROS graph for sensors, TF, joint states,
  and Nav2/localization.
- The active Gazebo ROS 2 controller set should be unambiguous: only
  `joint_state_broadcaster` should be active.

The final implementation meets that goal with native Gazebo `MecanumDrive`,
read-only `gz_ros2_control`, a `/cmd_vel` watchdog, and ROS-side wheel-state
odometry.

## Commit timeline

The branch contains these commits on top of `main`:

| Commit | Summary | Role in the branch |
| --- | --- | --- |
| `c097f4b` | Fix TF_OLD_DATA, controller loading, odometry; add project context | Stabilized early Fortress launch behavior, fixed controller activation race, and corrected holonomic odometry integration. |
| `7408fab` | Fix Fortress-only URDF, physics, deps, scripts, and launch cleanup | Removed Classic Gazebo assumptions, converted the sim toward Fortress-only dependencies and URDF/SDF patterns. |
| `86c0ab9` | Fix TF_OLD_DATA spam, RViz frame, camera plugin, and world sensors | Addressed DDS/TF spam and replaced Classic camera plumbing with native Fortress sensors and world systems. |
| `f85dba0` | Fix IMU and LiDAR sensors for Gazebo Fortress | Replaced Classic IMU/LiDAR plugins with native Fortress sensor definitions and bridge topics. |
| `1072f61` | Fix LiDAR queue spam, twist_to_stamped shutdown, and stale-process restart | Reduced LiDAR overload, cleaned Ctrl+C shutdown, and added stale-process cleanup to the launch helper. |
| `62d4218` | Fix double-robot spawn and LiDAR TF gaps | Identified ghost DDS `robot_state_publisher` / stale entity problems that caused duplicate robots and conflicting TF. |
| `dbf598b` | Fix double-spawn: use create -string to bypass DDS ghost RSP | Replaced `ros_gz_sim create -topic` with `create -string` so stale transient-local robot descriptions cannot spawn a second robot. |
| `5d1e4b0` | Fix mecanum strafing: remove gz:expressed_in, add slip2 | Tried world-frame `fdir1` plus slip as a workaround for Fortress friction direction issues. This was partial and later rejected. |
| `5ab4836` | Update CLAUDE.md: document friction fix, TODO list | Captured the state of the investigation and the known remaining friction/strafe questions. |
| `9d47d00` | Fix mecanum strafing: remove use_sim_time from twist_to_stamped, restore gz:expressed_in | Found a real timestamp bug in `twist_to_stamped`, but the broader `gz_ros2_control` strafe problem was not fully solved. |
| `e2c6fea` | Fix mecanum strafing: VelocityControl plugin bypasses DART expressed_in one-shot bug | Tested a body/world velocity control workaround. It moved correctly but did not satisfy the physics-based requirement. |
| `ce1304a` | Investigate mecanum strafing: document DART physics limitations | Reverted the body-velocity workaround, documented failed physics approaches, and isolated the difference between the local stack and Gazebo's official demo. |
| `2164e22` | Fix Gazebo mecanum strafing with native drive | Final working implementation: native `MecanumDrive`, read-only `gz_ros2_control`, command watchdog, wheel-state odometry, and corrected wheel friction SDF. |

## Main errors and discoveries

### Sim time and TF startup

Early launch attempts produced `TF_OLD_DATA` warnings and unstable TF. The
causes were layered:

- `robot_state_publisher` could start before Gazebo `/clock` existed, causing
  wall-clock-stamped TF to enter the TF buffer.
- Controller code that used `get_clock()->now()` could also see wall time before
  sim time settled.
- High-frequency joint state publication under load could reorder messages and
  make `robot_state_publisher` publish decreasing timestamps.

The branch addressed these by delaying startup where needed, using controller
update timestamps correctly, and capping `joint_state_broadcaster` publication
at 30 Hz.

### Controller loading race

The original controller loading path had a Humble lifecycle race: loading a
controller and then setting state separately could fail or leave controllers
inactive. The branch moved toward atomic activation and finally uses the
`controller_manager spawner` for `joint_state_broadcaster`, with longer service
timeouts to tolerate GUI starts.

### Classic Gazebo plugins in Fortress

Several Classic Gazebo plugins were present in the URDF and silently failed in
Fortress:

- `libgazebo_ros_camera.so`
- `libgazebo_ros_imu_sensor.so`
- `libgazebo_ros_ray_sensor.so`

The branch replaced these with native Fortress sensor definitions and added the
needed world systems (`Sensors`, `Imu`, and related bridge entries).

### Stale Gazebo and ghost DDS state

Repeated testing exposed stale process and transient-local DDS problems:

- Old Gazebo processes could leave a robot entity in the world.
- Ghost `robot_state_publisher` participants could provide old
  `robot_description` data to new subscribers.
- `ros_gz_sim create -topic` could therefore spawn a second robot or a renamed
  robot (`rosmaster_x3_0`), producing duplicate controller managers and TF
  conflicts.

The robust fix was to expand xacro once and spawn with `create -string`, avoiding
the DDS `robot_description` subscription entirely.

### Command timestamp timeout

One real no-motion bug came from `twist_to_stamped.py` using sim time in an
`ExecuteProcess` subprocess. In that path the ROS clock could publish
`stamp=0`, making the mecanum controller treat every command as older than its
0.5 s timeout and brake continuously.

Removing sim-time stamping from that converter was a valid fix for that
intermediate architecture, but the converter was later removed when the final
native Gazebo drive path was adopted.

### Failed mecanum physics approaches

Several approaches were tested before the final solution:

- Passive roller geometry: DART did not resolve passive roller DOFs in a way that
  produced realistic lateral mecanum forces.
- Bullet physics: the available Fortress Bullet path did not honor the needed
  `mu2` / `fdir1` friction behavior.
- `gz_ros2_control` plus `mecanum_drive_controller`: wheels spun and wheel odom
  moved, but the chassis did not physically strafe correctly.
- World-frame `fdir1` with slip tuning: could improve one orientation but did
  not preserve body-frame holonomic behavior after rotation.
- `VelocityControl`: produced motion, but it bypassed the intended wheel-ground
  force model by controlling body velocity.
- `OdometryPublisher`: rejected because it publishes ground-truth world pose,
  which hides wheel slip and contradicts the desired realistic odometry error.

The official Gazebo mecanum demo was the key comparison point. It used the
native `MecanumDrive` system and a spherical wheel collision with diagonal
anisotropic friction. That shifted the final architecture toward native
`MecanumDrive` for wheel commands while retaining ROS-side wheel-state odometry.

## Final architecture

### Command path

The final command path is:

```text
/cmd_vel
  -> cmd_vel_watchdog.py
  -> /cmd_vel_gz
  -> ros_gz_bridge
  -> /model/rosmaster_x3/cmd_vel
  -> Gazebo MecanumDrive
  -> wheel joint velocity commands
  -> DART wheel-ground contact forces
```

`MecanumDrive` owns wheel velocity commands. `gz_ros2_control` no longer exports
wheel velocity command interfaces, so there is no joint command ownership race.

### Odometry path

The final odometry path is:

```text
/joint_states
  -> wheel_state_odometry.py
  -> /odom
  -> /tf odom -> base_footprint
```

This keeps odometry encoder-like. If the wheels slip, the simulated body pose can
diverge from `/odom`, which is the intended behavior for navigation tuning.

### Wheel contact model

The final working wheel friction details are important:

- The wheel collision is a sphere, giving a single contact point.
- The `<surface>` block is nested under the wheel `<collision>` in the Gazebo
  extension.
- `fdir1` uses `ignition:expressed_in`, not `gz:expressed_in`, in this Fortress
  path.
- `fdir1` is expressed in `base_footprint`, because fixed-joint lumping makes
  `base_footprint` the model root body in the spawned SDF.
- Diagonal directions match the official Gazebo demo:
  - front-left and back-right: `1 -1 0`
  - front-right and back-left: `1 1 0`

The expanded spawned SDF confirmed that `base_link` becomes a frame attached to
`base_footprint`, not the root free body. This is why `base_link` looked right in
URDF but did not produce correct strafe in physics.

## Final changed surface

The final branch changes include:

- Fortress-only Gazebo launch cleanup.
- Native Fortress sensors and bridge entries for RGB-D, LiDAR, and IMU.
- More stable TF/controller startup.
- `create -string` model spawning to avoid ghost DDS robot descriptions.
- Native Gazebo `MecanumDrive` plugin in the robot xacro.
- Read-only `gz_ros2_control` wheel state interfaces.
- Removal of `mecanum_drive_controller` from Gazebo controller loading.
- `/cmd_vel_gz` bridge for the internal Gazebo command topic.
- `cmd_vel_watchdog.py` for the missing Fortress `MecanumDrive` timeout.
- `wheel_state_odometry.py` for encoder-style `/odom` and `odom -> base_footprint`.
- EKF input changed to `/odom`.
- Documentation in `README.md` and
  `yahboom_rosmaster_gazebo/doc/native_mecanum_drive_implementation.md`.

## Verification summary

The final implementation was verified in isolated Gazebo/ROS partitions to avoid
stale default-domain processes.

Final physical checks:

- `linear.y = 0.3` for about 5 seconds:
  - Gazebo body pose moved left about `1.3593 m`.
  - Wheel odometry reported about `1.3593 m`.
  - Yaw drift was essentially zero.
- `linear.x = 0.3` moved forward and matched wheel odometry.
- `angular.z = 0.5` rotated the robot in place.
- Stopping command publication caused the watchdog to zero wheel velocities.
- `ros2 control list_controllers` showed only `joint_state_broadcaster` active.
- GUI teleop with `teleop_twist_keyboard` worked, including left/right strafe.

Build/static checks used:

```bash
colcon build --symlink-install --packages-select yahboom_rosmaster_description yahboom_rosmaster_gazebo
python3 -m py_compile yahboom_rosmaster_gazebo/scripts/cmd_vel_watchdog.py yahboom_rosmaster_gazebo/scripts/wheel_state_odometry.py
git diff --check
```

## Remaining risks and follow-ups

- The final odometry intentionally has no added artificial encoder noise. Slip is
  represented because odometry comes from wheel joint states while the chassis
  moves through contact physics.
- Additional real-robot noise can be added later through wheel radius
  multipliers, encoder quantization, bias, or covariance parameters in
  `wheel_state_odometry.py`.
- `MecanumDrive`'s own Fortress odometry/TF topics are not used because this
  installed plugin advertises strings but did not publish usable odom/TF during
  testing.
- If Gazebo or gz-sim versions change, re-test the `ignition:expressed_in`
  behavior. This branch targets ROS 2 Humble and Gazebo Fortress 6.16.
