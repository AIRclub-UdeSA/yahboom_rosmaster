# Sourced by pixi on environment activation (macOS / RoboStack path).

# Gazebo finds system plugins through IGN_GAZEBO_SYSTEM_PLUGIN_PATH. The launch
# file seeds that from LD_LIBRARY_PATH, which conda environments never set and
# macOS does not use, so gz_ros2_control-system would fail to load. Point it at
# the environment's own lib directory instead.
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${CONDA_PREFIX}/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${CONDA_PREFIX}/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# A Mac exposes ~20 multicast-capable interfaces (en0, awdl0, bridge0, utun0-5).
# DDS participants pick different ones and then never discover each other, so
# nodes see a partial ROS graph: gz_ros2_control waits forever on
# robot_state_publisher and the controller spawner never finds
# /controller_manager. Pinning discovery to loopback makes the graph whole.
# Set unconditionally: the ROS activation that runs before this one already
# exports ROS_LOCALHOST_ONLY=0, so a :- default would never apply.
# Export ROS_LOCALHOST_ONLY=0 after activation to talk to a physical robot.
export ROS_LOCALHOST_ONLY=1

# Gazebo's own transport runs a separate discovery from DDS and picks an
# interface the same unreliable way. Pin it to loopback so `ign topic`,
# `ign service` and the GUI<->server link stay consistent.
export IGN_IP=127.0.0.1
export GZ_IP=127.0.0.1

# Overlay the colcon workspace once it has been built; a fresh checkout
# (pre-build) activates cleanly too.
if [ -f "${PIXI_PROJECT_ROOT}/install/setup.sh" ]; then
  . "${PIXI_PROJECT_ROOT}/install/setup.sh"
fi
