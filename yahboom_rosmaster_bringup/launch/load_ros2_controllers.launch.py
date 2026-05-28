#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the mecanum wheel robot.

Starts joint_state_broadcaster first, then mecanum_drive_controller once the
broadcaster is confirmed running.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart


def generate_launch_description():

    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    start_mecanum_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mecanum_drive_controller'],
        output='screen'
    )

    # Wait for the broadcaster to start, then activate the mecanum controller
    load_mecanum_on_broadcaster_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_joint_state_broadcaster_cmd,
            on_start=[
                TimerAction(period=2.0, actions=[start_mecanum_drive_controller_cmd])
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(start_joint_state_broadcaster_cmd)
    ld.add_action(load_mecanum_on_broadcaster_start)

    return ld
