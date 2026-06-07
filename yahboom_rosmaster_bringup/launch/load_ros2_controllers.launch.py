#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the mecanum wheel robot.

Starts only joint_state_broadcaster for the native Gazebo MecanumDrive simulator.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Generate the controller launch description."""
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_joint_state_broadcaster_cmd)

    return ld
