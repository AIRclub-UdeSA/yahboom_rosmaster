# yahboom_rosmaster

![OS](https://img.shields.io/badge/Ubuntu-22.04-orange)
![ROS_2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress%206-blue)

ROS 2 Humble support for the **ROSMASTER X3** mecanum wheel robot by Yahboom.

> This repository is forked from [automaticaddison/yahboom_rosmaster](https://github.com/automaticaddison/yahboom_rosmaster) (Jazzy) and adapted for ROS 2 Humble with **Gazebo Fortress only**.
> Gazebo Classic was dropped because its ODE engine cannot simulate holonomic mecanum motion correctly ([known fdir1 bug](https://github.com/gazebosim/gazebo-classic/issues/463)).

![ROSMASTER X3 in Gazebo](https://automaticaddison.com/wp-content/uploads/2024/11/gazebo-800-square-mecanum-controller.gif)

## Features

- **Mecanum wheel robot** with holonomic (omnidirectional) movement
- **Gazebo Fortress 6** — physics-based mecanum drive using DART engine with sphere wheel
  collisions and `gz:expressed_in="base_link"` friction direction locking for correct holonomic strafing
- **Sensors**: RGB-D Camera, 2D LiDAR, IMU
- **ROS 2 Control** integration via `gz_ros2_control`
- **Nav2 & SLAM** ready configuration
- Multiple world files for testing

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress 6

### Install Gazebo Fortress

```bash
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos
```

## Installation

```bash
# Create workspace
mkdir -p ~/rosmaster_ws/src
cd ~/rosmaster_ws/src

# Clone the repository
git clone https://github.com/juan-kaplan/yahboom_rosmaster.git

# Install dependencies
cd ~/rosmaster_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

## Quick Start

```bash
# Launch with RViz
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py

# Without RViz
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py rviz:=false
```

Verify the controllers loaded successfully (~10 seconds after launch):

```bash
ros2 control list_controllers
# Expected output:
#   joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
#   mecanum_drive_controller[mecanum_drive_controller/MecanumDriveController] active
```

### Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keys: `u i o`, `j k l`, `m , .` to move. **Shift+J / Shift+L** strafes left/right (holonomic).

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty.world` | Path to the Gazebo world file |
| `rviz` | `true` | Launch RViz automatically |
| `use_sim_time` | `true` | Use simulation time |

## Available Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (converted to TwistStamped internally) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/imu/data` | `sensor_msgs/Imu` | IMU readings |
| `/cam_1/color/image_raw` | `sensor_msgs/Image` | RGB camera |
| `/cam_1/depth/image_raw` | `sensor_msgs/Image` | Depth image |
| `/mecanum_drive_controller/odom` | `nav_msgs/Odometry` | Odometry |

## Physics Notes

The simulation uses the DART physics engine with:
- **Sphere wheel collisions** — required for a single contact point so that `gz:expressed_in="base_link"` locks the friction direction to the chassis frame as the wheel spins
- **Normalized fdir1 vectors** — FL/BR diagonal: `(0.707, -0.707, 0)`, FR/BL diagonal: `(0.707, 0.707, 0)`
- **Tuned contact parameters** — `kp=200 000`, `kd=2000` (reduces contact oscillations for 50 g wheels)
- **Symmetric ground friction** — `mu=mu2=100` (avoids traction bias between strafing and forward motion)

Odometry is computed from wheel encoder velocities in closed-loop mode. To model real-world slip and noise for navigation tuning, adjust `pose_covariance_diagonal` and `twist_covariance_diagonal` in `yahboom_rosmaster_gazebo/config/ros2_control.yaml`.

## Packages

| Package | Description |
|---------|-------------|
| `yahboom_rosmaster` | Metapackage |
| `yahboom_rosmaster_description` | URDF, meshes, RViz configs |
| `yahboom_rosmaster_gazebo` | Simulation launch files, worlds |
| `yahboom_rosmaster_navigation` | Nav2 configuration |
| `yahboom_rosmaster_localization` | Localization (robot_localization) |
| `yahboom_rosmaster_bringup` | Robot bringup launch files |
| `yahboom_rosmaster_docking` | Docking functionality |
| `yahboom_rosmaster_msgs` | Custom messages |

## License

BSD-3-Clause
