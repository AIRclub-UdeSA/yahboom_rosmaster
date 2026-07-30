#!/usr/bin/env bash
# Tear down every process a simulation launch starts.
#
# ros2 launch does not always reap its children on macOS, and a leftover
# robot_state_publisher creates a duplicate node name that breaks ROS graph
# discovery for the next run in ways that look like a networking fault.
set -u

PATTERNS=(
  "ros2 launch"
  "ign gazebo"
  "gzserver"
  "gzclient"
  "spawn_entity"
  "async_slam_toolbox_node"
  "controller_server"
  "planner_server"
  "bt_navigator"
  "behavior_server"
  "smoother_server"
  "velocity_smoother"
  "waypoint_follower"
  "lifecycle_manager"
  "ros_gz_sim/create"
  "robot_state_publisher"
  "parameter_bridge"
  # topic_tools installs the throttle as a bare binary, so it matches neither
  # "ros2 launch" nor its own node name without this.
  "topic_tools/throttle"
  "joint_state_throttle"
  "ros_gz_image/image_bridge"
  "cmd_vel_watchdog.py"
  "wheel_state_odometry.py"
  "controller_manager spawner"
  "rviz2"
)

for pattern in "${PATTERNS[@]}"; do
  pkill -f "$pattern" 2>/dev/null
done

sleep 2

for pattern in "${PATTERNS[@]}"; do
  pkill -9 -f "$pattern" 2>/dev/null
done

remaining=0
for pattern in "${PATTERNS[@]}"; do
  count=$(pgrep -f "$pattern" 2>/dev/null | wc -l | tr -d ' ')
  remaining=$((remaining + count))
done

echo "simulation stopped; ${remaining} matching processes remain"
