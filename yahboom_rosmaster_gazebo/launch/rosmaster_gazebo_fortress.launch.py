#!/usr/bin/env python3
"""
Launch Gazebo Fortress simulation for ROSMASTER X3 with physics-based mecanum drive.

Uses the native Gazebo MecanumDrive system for wheel velocity commands, with
gz_ros2_control kept read-only for joint states and wheel-link TF.
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _launch_rviz(context):
    launch_rviz = context.launch_configurations.get("rviz", "true")
    if launch_rviz.lower() in ("true", "1", "yes"):
        pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")
        default_rviz = os.path.join(pkg_gz, "rviz", "gazebo.rviz")
        use_sim_time = context.launch_configurations.get("use_sim_time", "true")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", default_rviz],
            parameters=[{"use_sim_time": use_sim_time == "true"}],
            output="screen",
        )
        return [TimerAction(period=5.0, actions=[rviz_node])]
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_desc = get_package_share_directory("yahboom_rosmaster_description")
    pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")

    default_world = os.path.join(pkg_gz, "worlds", "empty.world")
    default_xacro = os.path.join(pkg_desc, "urdf", "robots", "rosmaster_x3.urdf.xacro")
    bridge_config = os.path.join(pkg_gz, "config", "ros_gz_bridge.yaml")
    cmd_vel_watchdog_script = os.path.join(pkg_gz, "scripts", "cmd_vel_watchdog.py")
    wheel_odometry_script = os.path.join(pkg_gz, "scripts", "wheel_state_odometry.py")

    # Expand xacro once at launch-description time and share the string with both RSP
    # and the spawn node. Using -string (not -topic) for spawn means the create node
    # never opens a TRANSIENT_LOCAL DDS subscriber, so ghost RSP nodes from a previous
    # session cannot deliver a second robot_description and cause a double spawn.
    robot_description_str = subprocess.check_output([
        "xacro", default_xacro,
        "use_gazebo:=true",
        "robot_name:=rosmaster_x3",
        "prefix:=",
    ]).decode()

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_world = DeclareLaunchArgument("world", default_value=default_world)
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz (true/false)")
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Skip Gazebo GUI client — server-only for autonomous/CI debugging")
    headless = LaunchConfiguration("headless")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description_str,
        }],
    )

    # Gazebo Fortress server (headless)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments=[("gz_args", ["-r -s -v 4 ", world])],
    )

    # Gazebo Fortress GUI — skipped when headless:=true.
    # QT_QPA_PLATFORM=xcb forces X11/XWayland mode on Wayland sessions;
    # without it the Qt platform default fails on AMD Wayland, leaving a white window.
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments=[("gz_args", "-g")],
        condition=UnlessCondition(headless),
    )

    # Spawn robot directly from the pre-expanded URDF string.
    # z=0.0325 offsets base_footprint so wheel centres sit at wheel_radius above
    # ground — without this the roller collisions penetrate the ground plane.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description_str,
            "-name", "rosmaster_x3",
            "-z", "0.0325",
        ],
        output="screen",
    )

    # Bridge Gazebo command input and sensor topics.
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # Optimized image bridge for camera
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_1/image"],
        remappings=[("/cam_1/image", "/cam_1/color/image_raw")],
    )

    # Load and activate the read-only joint state broadcaster. The spawner waits
    # longer than `ros2 control load_controller`, which helps GUI starts on busy
    # machines where the controller manager is late to answer service calls.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Native MecanumDrive has no command timeout in Fortress 6.16, so keep the
    # public /cmd_vel contract and publish zero to the internal bridge topic when
    # commands stop.
    cmd_vel_watchdog = ExecuteProcess(
        cmd=["python3", cmd_vel_watchdog_script],
        output="screen",
    )

    # Encoder-style odometry from wheel joint states. This avoids Gazebo's
    # ground-truth OdometryPublisher while preserving /odom and odom->base TF.
    wheel_state_odometry = ExecuteProcess(
        cmd=["python3", wheel_odometry_script],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_rviz,
        declare_headless,
        # Force X11/XWayland for Gazebo GUI — prevents white window on Wayland + AMD GPU
        SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(pkg_gz, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_gz, "models")),
        gazebo_server,
        gazebo_client,
        # t=2s: RSP starts after /clock is available (avoids wall-clock TF poisoning).
        TimerAction(period=2.0, actions=[robot_state_publisher]),
        # t=3s: spawn using pre-expanded URDF string — no DDS topic subscription,
        # so ghost RSP nodes from a previous session are harmless.
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=5.0, actions=[
            ros_gz_bridge,
            ros_gz_image_bridge,
            cmd_vel_watchdog,
        ]),
        TimerAction(period=12.0, actions=[
            load_joint_state_broadcaster,
            wheel_state_odometry,
        ]),
        OpaqueFunction(function=_launch_rviz),
    ])
