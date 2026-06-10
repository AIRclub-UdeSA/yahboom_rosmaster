# yahboom_rosmaster

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress%206-blue)

ROS 2 Humble packages for the Yahboom ROSMASTER X3 mecanum robot, with a
Gazebo Fortress simulator, Nav2 configuration, localization, docking helpers,
and system-test utilities.

Gazebo Fortress is the supported simulator backend. Gazebo Classic is not
supported for the mecanum simulator.

## Repository Contents

| Package | Purpose |
|---------|---------|
| `yahboom_rosmaster` | Metapackage |
| `yahboom_rosmaster_description` | URDF, meshes, robot state publisher launch files, RViz configs |
| `yahboom_rosmaster_gazebo` | Gazebo Fortress worlds, bridge config, simulator launch, command watchdog, wheel odometry |
| `yahboom_rosmaster_bringup` | Combined simulator, localization, docking, and Nav2 launch files |
| `yahboom_rosmaster_navigation` | Nav2 parameters, maps, and navigation helper scripts |
| `yahboom_rosmaster_localization` | `robot_localization` EKF configuration and launch files |
| `yahboom_rosmaster_docking` | AprilTag docking support |
| `yahboom_rosmaster_msgs` | Custom messages |
| `yahboom_rosmaster_system_tests` | Small test/demo nodes for mecanum motion and services |

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress 6
- `colcon`
- `rosdep`

Install the common build tools first:

```bash
sudo apt update
sudo apt install -y \
  curl \
  gnupg \
  lsb-release \
  python3-colcon-common-extensions \
  python3-rosdep
```

Configure the OSRF Gazebo apt repository and install Gazebo Fortress:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y ignition-fortress
```

Install Gazebo Fortress ROS integration packages:

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-gz-ros2-control-demos
```

For headless launch helper scripts, install:

```bash
sudo apt install -y xvfb
```

## Build

```bash
mkdir -p ~/rosmaster_ws/src
cd ~/rosmaster_ws/src
git clone https://github.com/juan-kaplan/yahboom_rosmaster.git

cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

If `rosdep` has not been initialized on the machine:

```bash
sudo rosdep init
rosdep update
```

## Standalone Simulator

Launch Gazebo Fortress with RViz:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py
```

Launch headless, without RViz:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  rviz:=false \
  headless:=true
```

Use a specific world:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:=$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/cafe.world
```

### Simulator Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `worlds/empty.world` | Gazebo world path |
| `rviz` | `true` | Start RViz |
| `headless` | `false` | Run Gazebo server without the GUI client |
| `use_sim_time` | `true` | Use simulation time |
| `gz_args` | empty | Extra Gazebo Sim arguments |

## Nav2 Simulator Launch

Launch Gazebo, wheel odometry, EKF, docking support, and Nav2:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py
```

Headless Nav2 smoke launch:

```bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py \
  rviz:=false \
  headless:=true
```

The Nav2 launch defaults to:

- `yahboom_rosmaster_gazebo/worlds/cafe.world`
- `yahboom_rosmaster_navigation/maps/cafe_world_map.yaml`
- `use_sim_time:=true`
- `autostart:=true`

## Command And State Flow

The public command API is `/cmd_vel`.

```text
/cmd_vel
  -> cmd_vel_watchdog.py
  -> /cmd_vel_gz
  -> ros_gz_bridge
  -> /model/rosmaster_x3/cmd_vel
  -> Gazebo MecanumDrive
```

The simulator uses Gazebo's native `MecanumDrive` system for wheel commands.
`gz_ros2_control` is read-only for wheel state, and only
`joint_state_broadcaster` is loaded.

Odometry and TF ownership:

- `/joint_states` is published by `joint_state_broadcaster`.
- `/odom` is published by `wheel_state_odometry.py`.
- `odom -> base_footprint` is published by `wheel_state_odometry.py`.
- The Gazebo EKF configuration does not publish TF.

## Main Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Public velocity command input |
| `/cmd_vel_gz` | `geometry_msgs/msg/Twist` | Internal watchdog output bridged to Gazebo |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint state from `joint_state_broadcaster` |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel-state odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static robot transforms |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU |
| `/cam_1/color/image_raw` | `sensor_msgs/msg/Image` | RGB camera image |
| `/cam_1/depth/image_raw` | `sensor_msgs/msg/Image` | Depth image |

## Teleoperation

Install teleop if needed:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

Run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The simulator listens on `/cmd_vel`. Use the standard
`teleop_twist_keyboard` holonomic bindings for mecanum strafing.

## Smoke Tests

After launching the standalone simulator or Nav2 simulator, run these checks in
a second terminal:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 topic list | sort | grep -E 'cmd_vel|odom|joint_states|tf|map|mecanum'
```

Expected controller:

```text
joint_state_broadcaster ... active
```

The old controller command topic should not exist:

```bash
ros2 topic list | grep '^/mecanum_drive_controller/cmd_vel$' \
  && echo "BAD: old controller topic exists" \
  || echo "OK: old controller topic absent"
```

Check odometry and TF:

```bash
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /odom --once
```

Publish a forward command:

```bash
ros2 topic pub --rate 10 --times 15 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic echo /odom --once
```

After the watchdog timeout, `/cmd_vel_gz` should return to zero:

```bash
sleep 1
ros2 topic echo /cmd_vel_gz --once
```

## Development Checks

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

git -C src/yahboom_rosmaster diff --check

python3 -m py_compile $(find src/yahboom_rosmaster -name '*.py' \
  -not -path '*/build/*' \
  -not -path '*/install/*' \
  -not -path '*/log/*')

xacro src/yahboom_rosmaster/yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro \
  use_gazebo:=true > /tmp/rosmaster_x3.urdf
check_urdf /tmp/rosmaster_x3.urdf

colcon build --symlink-install
colcon test
colcon test-result --verbose --all
rosdep check --from-paths src --ignore-src --rosdistro humble
```

## Troubleshooting

### Stale Overlay

If a deleted package such as `mecanum_drive_controller` still appears, or a
new package cannot be found, clean and rebuild the workspace root:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

Check package resolution:

```bash
ros2 pkg prefix yahboom_rosmaster_docking
ros2 pkg prefix yahboom_rosmaster_navigation
ros2 pkg prefix mecanum_drive_controller
```

`mecanum_drive_controller` should not resolve from the workspace install.

### Early Nav2 TF Warnings

During Nav2 startup, costmaps may briefly report that `odom` is unavailable
while Gazebo spawns the robot and `joint_state_broadcaster` starts. The expected
steady state is:

```text
wheel_state_odometry: Publishing wheel-state odometry from /joint_states to /odom
spawner_joint_state_broadcaster: Configured and activated joint_state_broadcaster
lifecycle_manager_navigation: Managed nodes are active
```

## Additional Documentation

- [Native mecanum drive implementation](yahboom_rosmaster_gazebo/doc/native_mecanum_drive_implementation.md)

## License

BSD-3-Clause
