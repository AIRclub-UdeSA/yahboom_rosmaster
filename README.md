# yahboom_rosmaster

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress%206-blue)

ROS 2 Humble packages for simulating the Yahboom ROSMASTER X3 mecanum robot
with Gazebo Fortress. The supported standalone workflow provides contact-driven
holonomic motion, wheel-state odometry, TF, 2D LiDAR, IMU data, and a depth
point cloud, along with RGB and depth camera images and camera calibration
messages.

Gazebo Fortress is the supported simulator backend. Gazebo Classic is not
supported by the current mecanum simulator.

## Requirements

- Ubuntu 22.04
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Gazebo Fortress 6
- `git`, `colcon`, and `rosdep`

ROS 2 Humble and Gazebo Fortress are the supported ROS/Gazebo pairing. After
installing ROS 2 Humble, install the common workspace tools and ROS-Gazebo
integration packages:

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-ros-gz
```

The build instructions below use `rosdep` to install the remaining dependencies
declared by the repository packages.

## Build

Create a workspace and clone the repository:

```bash
mkdir -p ~/rosmaster_ws/src
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/yahboom_rosmaster.git
```

Initialize `rosdep` once on a new machine:

```bash
sudo rosdep init
```

If `rosdep` is already initialized, skip that command. Then install dependencies
and build from the workspace root:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --symlink-install
source install/setup.bash
```

Source both ROS 2 and the workspace overlay in every new terminal used with the
simulator:

```bash
source /opt/ros/humble/setup.bash
source ~/rosmaster_ws/install/setup.bash
```

## Quick Start

### Launch the Simulator

Start the default empty world with the Gazebo GUI and RViz:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py
```

Startup is staged while Gazebo creates the robot and starts its ROS interfaces.
Wait for these messages before checking odometry:

```text
Configured and activated joint_state_broadcaster
Publishing wheel-state odometry from /joint_states to /odom
```

### Launch Without GUIs

Run the Gazebo server without the Gazebo GUI or RViz:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  rviz:=false \
  headless:=true
```

### Launch the Cafe World

The repository supports the empty and cafe Fortress worlds. Launch the cafe
world with:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/cafe.world"
```

### Simulator Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `worlds/empty.world` | Absolute path to the Gazebo world file |
| `rviz` | `true` | Start RViz |
| `headless` | `false` | Run the Gazebo server without its GUI client |
| `use_sim_time` | `true` | Use the Gazebo simulation clock; keep enabled for the supported workflow |

## Controlling the Robot

The public velocity-command topic is `/cmd_vel`. Positive `linear.x` moves the
robot forward, positive `linear.y` strafes left, and positive `angular.z`
rotates counterclockwise.

### Keyboard Teleoperation

Install the keyboard teleoperation package if needed:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

In a second sourced terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Follow the program's holonomic movement bindings to drive and strafe the robot.

### Direct Motion Commands

The following finite commands are useful for checking each mecanum axis. Run
them one at a time with enough free space around the robot.

Move forward for approximately two seconds:

```bash
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Strafe left for approximately two seconds:

```bash
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Rotate counterclockwise for approximately two seconds:

```bash
ros2 topic pub --rate 10 --times 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.50}}'
```

## Simulator Architecture

### Command Flow

```text
/cmd_vel
  -> cmd_vel_watchdog.py
  -> /cmd_vel_gz
  -> ros_gz_bridge
  -> /model/rosmaster_x3/cmd_vel
  -> Gazebo MecanumDrive
  -> wheel joint velocity targets
  -> DART wheel/ground contact
```

Gazebo's native `MecanumDrive` system calculates the four wheel targets.
`gz_ros2_control` is kept read-only for wheel and IMU state, and only
`joint_state_broadcaster` is loaded.

The watchdog republishes the latest command to the internal `/cmd_vel_gz` topic
and publishes zero when `/cmd_vel` has been silent for 0.5 seconds.

### Odometry and TF

- `/joint_states` is published by `joint_state_broadcaster`.
- `/odom` is integrated from wheel joint positions by
  `wheel_state_odometry.py`.
- `odom -> base_footprint` is published by `wheel_state_odometry.py`.
- Robot link transforms are published by `robot_state_publisher`.

## Working ROS Interfaces

| Topic | Type | Purpose |
|-------|------|---------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation clock |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Public velocity-command input |
| `/cmd_vel_gz` | `geometry_msgs/msg/Twist` | Internal watchdog output bridged to Gazebo |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel joint positions and velocities |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel-state odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static robot transforms |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR scan |
| `/imu/data` | `sensor_msgs/msg/Imu` | Simulated IMU data |
| `/cam_1/color/image_raw` | `sensor_msgs/msg/Image` | RGB camera image |
| `/cam_1/depth/image_raw` | `sensor_msgs/msg/Image` | Depth camera image |
| `/cam_1/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB camera intrinsics |
| `/cam_1/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth camera intrinsics |
| `/cam_1/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Depth-camera point cloud |

## Verify the Simulator

Run these checks in a second sourced terminal after simulator startup.

### Controller and Topics

```bash
ros2 control list_controllers

ros2 topic list | sort | grep -E \
  '^/(clock|cmd_vel|cmd_vel_gz|joint_states|odom|scan|imu/data|tf|tf_static|cam_1/)'
```

The controller list should contain:

```text
joint_state_broadcaster ... active
```

The removed custom controller topic should not exist:

```bash
ros2 topic list | grep '^/mecanum_drive_controller/cmd_vel$' \
  && echo "BAD: removed controller topic exists" \
  || echo "OK: removed controller topic is absent"
```

### Odometry and TF

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
timeout --signal=INT 5 ros2 run tf2_ros tf2_echo odom base_footprint
```

### Sensors

Each command should report incoming messages:

```bash
timeout --signal=INT 5 ros2 topic hz /scan
timeout --signal=INT 5 ros2 topic hz /imu/data
timeout --signal=INT 5 ros2 topic hz /cam_1/color/image_raw
timeout --signal=INT 5 ros2 topic hz /cam_1/depth/image_raw
timeout --signal=INT 5 ros2 topic hz /cam_1/depth/color/points
ros2 topic echo /cam_1/color/camera_info --once
ros2 topic echo /cam_1/depth/camera_info --once
```

The repository also registers an end-to-end sensor contract for both supported
worlds. It validates message delivery and the expected clock, frame IDs, image
encodings, camera dimensions and intrinsics, point fields, LiDAR metadata, IMU
values, wheel joints, odometry, and TF:

```bash
colcon test --packages-select yahboom_rosmaster_gazebo \
  --ctest-args -R sensor_contract_ --output-on-failure
colcon test-result --verbose
```

### Command Watchdog

Publish one nonzero input, wait longer than the 0.5-second timeout, and inspect
the internal command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

sleep 1
ros2 topic echo /cmd_vel_gz --once
```

The reported twist should be zero. The robot's physical stopping time is also
affected by the acceleration limit in the Gazebo drive plugin.

## Development Checks

Run the repository checks from the workspace root:

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

### Package or Launch File Not Found

Source both the ROS installation and the workspace overlay in the current
terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rosmaster_ws/install/setup.bash
```

Confirm that ROS resolves the simulator package from the expected workspace:

```bash
ros2 pkg prefix yahboom_rosmaster_gazebo
```

### Controller or Odometry Not Ready

Robot creation and controller startup are staged. Wait for the controller
activation message, then check:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
```

Also confirm that Gazebo is running and the simulation is not paused.

### Stale Workspace Overlay

If a deleted package such as `mecanum_drive_controller` still resolves, or a new
package cannot be found, rebuild a clean workspace overlay:

```bash
cd ~/rosmaster_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

The removed controller package should not resolve from the rebuilt workspace:

```bash
ros2 pkg prefix mecanum_drive_controller
```

### Run Without the Gazebo GUI

If the Gazebo GUI cannot start in the current display environment, use the
server-only command:

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  rviz:=false \
  headless:=true
```

## Current Project Status

The supported user path is the standalone, single-robot Fortress simulator in
the empty or cafe world. Its nominal forward, lateral, and rotational motion,
wheel odometry, TF, LiDAR, IMU, RGB and depth images, camera information, and
depth point cloud have been exercised on ROS 2 Humble. Automated headless
sensor contracts cover both supported worlds.

The following repository surfaces are retained for continued development but
are not part of the supported workflow described above:

- The combined Nav2 launch loads its nodes, but its simulation-time propagation
  is not currently reliable enough for navigation-goal execution.
- AprilTag and docking resources are present, but they do not provide a working
  end-to-end docking workflow.
- The drivetrain is an idealized contact-driven model. It is not calibrated to
  reproduce measured motor, encoder, wheel, floor, latency, or battery error
  from a physical ROSMASTER X3.
- Sensor data is nominal simulation output. The camera, LiDAR, and IMU models
  have not been calibrated against measurements from the physical robot.
- `simple_room.world` and `willowgarage.world` are retained migration assets;
  they are not supported Fortress worlds.
- Multi-robot operation and real-hardware bringup are not provided.
- The registered tests include lint, style, XML, and the empty/cafe runtime
  sensor contracts. Motion behavior still uses the manual checks above.

The 0.5-second watchdog handles normal command loss. It is not a safety-rated
controller: terminating the watchdog or its bridge can leave Gazebo retaining
the last drive target until another command is received or the simulation is
stopped.

## Repository Layout

| Package | Contents |
|---------|----------|
| `yahboom_rosmaster` | Repository metapackage |
| `yahboom_rosmaster_description` | Xacro/URDF, meshes, robot-state launch files, and RViz configuration |
| `yahboom_rosmaster_gazebo` | Fortress worlds, bridge configuration, simulator launch, command watchdog, and wheel odometry |
| `yahboom_rosmaster_bringup` | Integration launch files and command-line helpers |
| `yahboom_rosmaster_navigation` | Navigation parameters, maps, and helper code |
| `yahboom_rosmaster_localization` | `robot_localization` EKF configuration and launch files |
| `yahboom_rosmaster_docking` | AprilTag and docking-related helper code |
| `yahboom_rosmaster_msgs` | Custom messages, service, and action definitions |
| `yahboom_rosmaster_system_tests` | Manual example and demo nodes for commands, messages, services, and actions |

## Provenance

This repository is a ROS 2 Humble and Gazebo Fortress fork of
[Automatic Addison's `yahboom_rosmaster`](https://github.com/automaticaddison/yahboom_rosmaster)
repository. The current fork is maintained at
[`AIRclub-UdeSA/yahboom_rosmaster`](https://github.com/AIRclub-UdeSA/yahboom_rosmaster).

## License

The packages are distributed under the BSD-3-Clause license. Each package
contains its own `LICENSE` file.
