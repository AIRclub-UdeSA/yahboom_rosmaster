#!/usr/bin/env python3
"""
Launch Gazebo Fortress simulation for ROSMASTER X3 with physics-based mecanum drive.

Uses the native Gazebo MecanumDrive system for wheel velocity commands, with
gz_ros2_control kept read-only for joint states and wheel-link TF.
"""
import os
import shutil
import subprocess
import time

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnShutdown
from launch.logging import get_logger
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


def _gazebo_process_stopped(gazebo_server):
    """Return whether the owned Gazebo process has exited or become a zombie."""
    if gazebo_server.return_code is not None:
        return True
    details = gazebo_server.process_details
    if details is None or "pid" not in details:
        return False
    try:
        with open(f"/proc/{details['pid']}/stat", encoding="utf-8") as stat_file:
            state = stat_file.read().rsplit(")", 1)[1].strip().split()[0]
    except FileNotFoundError:
        return True
    except (IndexError, OSError):
        return False
    return state == "Z"


def _request_gazebo_stop(event, context, gazebo_server):
    """Ask Gazebo to stop cleanly before launch falls back to process signals."""
    del event
    logger = get_logger("rosmaster_gazebo_shutdown")
    ign_executable = shutil.which("ign")
    if ign_executable is None:
        logger.warning("Cannot request Gazebo stop: 'ign' is not available")
        return None

    try:
        result = subprocess.run(
            [
                ign_executable,
                "service",
                "-s", "/server_control",
                "--reqtype", "ignition.msgs.ServerControl",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1500",
                "--req", "stop: true",
            ],
            capture_output=True,
            check=False,
            env=context.environment,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exception:
        logger.warning(
            f"Gazebo stop service was unavailable; using signal fallback: {exception}")
        return None

    if result.returncode != 0 or "data: true" not in result.stdout:
        detail = result.stderr.strip() or result.stdout.strip() or "no response"
        logger.warning(
            "Gazebo did not acknowledge the stop service; using signal fallback: "
            f"{detail}")
    else:
        logger.info("Gazebo acknowledged the clean stop request")
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if _gazebo_process_stopped(gazebo_server):
                logger.info("Gazebo completed its clean stop before signal fallback")
                break
            time.sleep(0.05)
        else:
            logger.warning(
                "Gazebo did not finish its service-requested stop within 4 seconds; "
                "using signal fallback")
    return None


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    pkg_desc = get_package_share_directory("yahboom_rosmaster_description")
    pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")
    ign_executable = shutil.which("ign")
    if ign_executable is None:
        raise RuntimeError("Could not find the Gazebo Fortress 'ign' executable")

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

    # Own the Ruby/Gazebo process directly so launch's SIGINT reaches it. The
    # Humble ros_gz_sim wrapper uses ExecuteProcess(shell=True), which signals a
    # waiting /bin/sh instead; after escalation that can orphan the real server.
    gazebo_server = ExecuteProcess(
        cmd=[
            "ruby", ign_executable, "gazebo",
            "-r", "-s", "-v", "4", world,
            "--force-version", "6",
        ],
        output="screen",
    )

    # Gazebo Fortress GUI — skipped when headless:=true.
    # QT_QPA_PLATFORM=xcb forces X11/XWayland mode on Wayland sessions;
    # without it the Qt platform default fails on AMD Wayland, leaving a white window.
    gazebo_client = ExecuteProcess(
        cmd=[
            "ruby", ign_executable, "gazebo", "-g",
            "--force-version", "6",
        ],
        output="screen",
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

    # Optimized image bridge for the native Fortress RGB-D camera outputs.
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_1/image", "/cam_1/depth_image"],
        remappings=[
            ("/cam_1/image", "/cam_1/color/image_raw"),
            ("/cam_1/depth_image", "/cam_1/depth/image_raw"),
        ],
        output="screen",
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
        # Match ros_gz_sim's plugin search environment for ROS-installed Gazebo
        # systems such as gz_ros2_control while bypassing its shell wrapper.
        AppendEnvironmentVariable(
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", os.environ.get("LD_LIBRARY_PATH", "")),
        AppendEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", os.environ.get("LD_LIBRARY_PATH", "")),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(pkg_gz, "models")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_gz, "models")),
        # Gazebo occasionally misses process-level SIGINT after sequential test
        # runs. Its control service cleanly stops the server first; launch's
        # normal SIGINT/SIGTERM escalation remains available as a fallback.
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: _request_gazebo_stop(
                event, context, gazebo_server))),
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
