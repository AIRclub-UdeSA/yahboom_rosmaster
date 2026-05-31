# Native MecanumDrive implementation

This note documents the Gazebo Fortress mecanum simulation change for the
ROSMASTER X3.

## Summary

The simulator now uses Gazebo Fortress's native `MecanumDrive` system to command
wheel joint velocities while keeping `gz_ros2_control` read-only for joint state
publication. ROS still exposes `/cmd_vel` as the public command topic, but a
watchdog node republishes it to the internal `/cmd_vel_gz` bridge topic and
sends zero commands after 0.5 seconds without input.

Odometry is produced on the ROS side from `/joint_states`, so `/odom` remains
encoder-style wheel odometry instead of Gazebo ground-truth pose.

## Root cause

The robot was receiving wheel commands, but the chassis did not strafe because
the wheel contact friction direction was not applied to the actual collision in
the spawned SDF.

The final working combination is:

- Put the wheel `<surface>` block inside `<gazebo reference="wheel"><collision>`.
- Use `ignition:expressed_in`, which is the namespace honored by Fortress here.
- Express `fdir1` in `base_footprint`, because the URDF fixed joints are lumped
  and `base_footprint` becomes the model root body in the spawned SDF.
- Use the same diagonal fdir pattern as the official Gazebo mecanum demo:
  front-left/back-right `1 -1 0`, front-right/back-left `1 1 0`.

Using `base_link` or `gz:expressed_in` looked valid in the xacro but did not
produce correct physical strafing after URDF-to-SDF conversion.

## Changed files

- `yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro`
  adds the native `ignition-gazebo-mecanum-drive-system` plugin with the X3
  wheel joints and geometry parameters.
- `yahboom_rosmaster_description/urdf/control/rosmaster_x3_ros2_control.urdf.xacro`
  removes wheel velocity command interfaces so `gz_ros2_control` only exposes
  wheel position and velocity state.
- `yahboom_rosmaster_description/urdf/mech/mecanum_wheel.urdf.xacro`
  attaches the anisotropic friction block to the wheel collision and locks
  `fdir1` to `base_footprint`.
- `yahboom_rosmaster_gazebo/config/ros2_control.yaml`
  removes `mecanum_drive_controller`; only `joint_state_broadcaster` remains.
- `yahboom_rosmaster_gazebo/config/ros_gz_bridge.yaml`
  bridges ROS `/cmd_vel_gz` to Gazebo `/model/rosmaster_x3/cmd_vel`.
- `yahboom_rosmaster_gazebo/launch/rosmaster_gazebo_fortress.launch.py`
  removes the old twist converter and mecanum controller loading, then launches
  the bridge, watchdog, wheel odometry node, image bridge, and joint state
  broadcaster spawner.
- `yahboom_rosmaster_gazebo/scripts/cmd_vel_watchdog.py`
  republishes `/cmd_vel` to `/cmd_vel_gz` at 30 Hz and sends zero after 0.5 s.
- `yahboom_rosmaster_gazebo/scripts/wheel_state_odometry.py`
  integrates wheel joint positions and publishes `/odom` plus
  `odom -> base_footprint` on `/tf`.
- `yahboom_rosmaster_gazebo/package.xml`
  adds runtime dependencies for the new Python nodes.
- `yahboom_rosmaster_localization/config/ekf.yaml`
  consumes `/odom` instead of the removed mecanum controller odometry topic.
- `README.md`
  documents the new command, odometry, controller, and physics interfaces.

## Public interfaces

- `/cmd_vel` remains the user and Nav2 command topic.
- `/cmd_vel_gz` is internal and bridged to Gazebo `MecanumDrive`.
- `/odom` is wheel-state odometry from `/joint_states`.
- `/tf` includes `odom -> base_footprint` from wheel odometry and robot link TF
  from `robot_state_publisher`.
- `ros2 control list_controllers` should show only
  `joint_state_broadcaster` active.

## Verification

The final isolated Fortress test used a private `ROS_DOMAIN_ID` and
`IGN_PARTITION` to avoid stale Gazebo processes. Results:

- `linear.y = 0.3` for about 5 seconds physically strafed left:
  Gazebo body pose `y = 1.359299773651377`, wheel odom
  `y = 1.359299998876786`, near-zero yaw.
- `linear.x = 0.3` moved forward and matched wheel odometry.
- `angular.z = 0.5` rotated the body in place.
- After command publication stopped, watchdog output zeroed the wheel velocities.
- `joint_state_broadcaster` was the only active controller.

Build and static checks passed:

```bash
colcon build --symlink-install --packages-select yahboom_rosmaster_description yahboom_rosmaster_gazebo
python3 -m py_compile yahboom_rosmaster_gazebo/scripts/cmd_vel_watchdog.py yahboom_rosmaster_gazebo/scripts/wheel_state_odometry.py
git diff --check
```

## Manual retest

```bash
pkill -f 'ros2 launch yahboom_rosmaster_gazebo|ign gazebo|parameter_bridge|cmd_vel_watchdog|wheel_state_odometry|twist_to_stamped|mecanum_drive_controller' || true

cd ~/Documents/rosmaster_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select yahboom_rosmaster_description yahboom_rosmaster_gazebo
source install/setup.bash

ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py headless:=false rviz:=false
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/rosmaster_ws/install/setup.bash

timeout 5 ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"
```

Positive ROS `linear.y` is left strafe. Use `linear.y: -0.3` for right strafe.
