#!/usr/bin/env python3
"""
Launch Gazebo Fortress simulation for ROSMASTER X3 with physics-based mecanum drive.

Uses the native Gazebo MecanumDrive system for wheel velocity commands, with
gz_ros2_control kept read-only for joint states and wheel-link TF.
"""
import os
import platform
import shutil
import subprocess
import tempfile
import time

import yaml

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


MOTION_PROFILE_KEYS = (
    "wheel_mu",
    "wheel_mu2",
    "wheel_slip2",
    "front_left_slip1",
    "front_right_slip1",
    "back_left_slip1",
    "back_right_slip1",
)


def _load_motion_profile(config_path, profile_name):
    """Load and validate one deterministic wheel-contact profile."""
    with open(config_path, encoding="utf-8") as profile_file:
        document = yaml.safe_load(profile_file)

    profiles = document.get("profiles", {}) if isinstance(document, dict) else {}
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles)) or "none"
        raise RuntimeError(
            f"Unknown motion profile '{profile_name}'; available profiles: {available}")

    profile = profiles[profile_name]
    missing = [key for key in MOTION_PROFILE_KEYS if key not in profile]
    extra = [key for key in profile if key not in MOTION_PROFILE_KEYS]
    if missing or extra:
        raise RuntimeError(
            f"Invalid motion profile '{profile_name}': "
            f"missing={missing}, extra={extra}")

    values = {}
    for key in MOTION_PROFILE_KEYS:
        try:
            values[key] = float(profile[key])
        except (TypeError, ValueError) as exception:
            raise RuntimeError(
                f"Motion profile '{profile_name}' value '{key}' must be numeric") \
                from exception
        if values[key] < 0.0:
            raise RuntimeError(
                f"Motion profile '{profile_name}' value '{key}' must be non-negative")
    return values


def _launch_robot(context, xacro_path, profile_config):
    """Expand the selected motion profile once for RSP and Gazebo spawn."""
    profile_name = LaunchConfiguration("motion_profile").perform(context)
    profile = _load_motion_profile(profile_config, profile_name)
    render_sensors = LaunchConfiguration("render_sensors").perform(context)
    if render_sensors.lower() not in ("true", "1", "yes"):
        get_logger("rosmaster_gazebo_render_sensors").warning(
            "Rendering sensors (LiDAR, RGB-D camera) are disabled; /scan and "
            "/cam_1 topics will stay silent")
    command = [
        "xacro", xacro_path,
        "use_gazebo:=true",
        "robot_name:=rosmaster_x3",
        "prefix:=",
        f"render_sensors:={render_sensors}",
        f"use_ros2_control:={LaunchConfiguration('use_ros2_control').perform(context)}",
    ]
    command.extend(f"{key}:={profile[key]}" for key in MOTION_PROFILE_KEYS)
    robot_description = subprocess.check_output(command, text=True)

    get_logger("rosmaster_gazebo_motion_profile").info(
        f"Using '{profile_name}' wheel-contact profile from {profile_config}")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time.lower() in ("true", "1", "yes"),
            "robot_description": robot_description,
        }],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description,
            "-name", "rosmaster_x3",
            "-z", "0.0325",
        ],
        output="screen",
    )
    return [
        # RSP starts after /clock is available, avoiding wall-clock TF poisoning.
        TimerAction(period=2.0, actions=[robot_state_publisher]),
        # Spawn from the same expanded string. The create node never subscribes
        # to robot_description, preventing stale transient-local double spawns.
        TimerAction(period=3.0, actions=[spawn]),
    ]


def _rviz_config_for_platform(default_rviz):
    """
    Return an RViz config the host can actually open.

    The Camera display builds a second OGRE render panel, which aborts RViz on
    macOS with "mutex lock failed". macOS has no simulated camera anyway, so
    drop that one display and keep a single config as the source of truth.
    """
    if platform.system() != "Darwin":
        return default_rviz

    with open(default_rviz, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    displays = config.get("Visualization Manager", {}).get("Displays", [])
    config["Visualization Manager"]["Displays"] = [
        display for display in displays
        if display.get("Class") != "rviz_default_plugins/Camera"
    ]

    patched = os.path.join(tempfile.gettempdir(), "rosmaster_gazebo_macos.rviz")
    with open(patched, "w", encoding="utf-8") as config_file:
        yaml.safe_dump(config, config_file, sort_keys=False)
    return patched


def _launch_rviz(context):
    launch_rviz = context.launch_configurations.get("rviz", "true")
    if launch_rviz.lower() in ("true", "1", "yes"):
        pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")
        default_rviz = _rviz_config_for_platform(
            os.path.join(pkg_gz, "rviz", "gazebo.rviz"))
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
    world = LaunchConfiguration("world")

    pkg_desc = get_package_share_directory("yahboom_rosmaster_description")
    pkg_gz = get_package_share_directory("yahboom_rosmaster_gazebo")
    ign_executable = shutil.which("ign")
    if ign_executable is None:
        raise RuntimeError("Could not find the Gazebo Fortress 'ign' executable")

    default_world = os.path.join(pkg_gz, "worlds", "empty.world")
    default_xacro = os.path.join(pkg_desc, "urdf", "robots", "rosmaster_x3.urdf.xacro")
    bridge_config = os.path.join(pkg_gz, "config", "ros_gz_bridge.yaml")
    motion_profile_config = os.path.join(pkg_gz, "config", "motion_profiles.yaml")
    motion_bias_config = os.path.join(pkg_gz, "config", "motion_bias.yaml")
    wheel_odometry_script = os.path.join(pkg_gz, "scripts", "wheel_state_odometry.py")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_world = DeclareLaunchArgument("world", default_value=default_world)
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz (true/false)")
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Skip Gazebo GUI client — server-only for autonomous/CI debugging")
    # The Gazebo Fortress server initialises ogre2 on its own render thread,
    # which macOS forbids (NSWindow is main-thread only) and which segfaults the
    # server as soon as a gpu_lidar or rgbd_camera spawns. Default those sensors
    # off on macOS so the rest of the simulation runs.
    declare_render_sensors = DeclareLaunchArgument(
        "render_sensors",
        default_value="false" if platform.system() == "Darwin" else "true",
        choices=["true", "false"],
        description=(
            "Spawn the rendering sensors (LiDAR, RGB-D camera). Unsupported by "
            "the Gazebo Fortress server on macOS"),
    )
    # Activating a controller calls ControllerManager::switch_controller, which
    # waits on a condition variable holding a deferred (unlocked) mutex. libc++
    # raises EPERM there inside a noexcept function, so the Gazebo server
    # aborts. Publish joint state directly from Gazebo on macOS instead.
    declare_use_ros2_control = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false" if platform.system() == "Darwin" else "true",
        choices=["true", "false"],
        description=(
            "Use gz_ros2_control for joint state. When false, Gazebo's "
            "JointStatePublisher feeds /joint_states through ros_gz_bridge"),
    )
    declare_motion_bias = DeclareLaunchArgument(
        "motion_bias",
        default_value="true",
        description=(
            "Add the per-direction drift of an uncalibrated mecanum base to "
            "/cmd_vel. Resampled at random per launch, so switch it off for "
            "tests that assert an exact trajectory"),
    )
    declare_motion_profile = DeclareLaunchArgument(
        "motion_profile",
        default_value="stress",
        choices=["ideal", "stress"],
        description=(
            "Wheel-contact profile: stress is deterministic and uncalibrated; "
            "ideal preserves the zero-slip baseline"),
    )
    headless = LaunchConfiguration("headless")

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
    # The Gazebo GUI cannot run on macOS: ign-gui builds its 3D scene on a
    # secondary thread, and creating the ogre2 render window there trips the
    # same NSWindow main-thread rule that blocks rendering sensors. The `ign
    # gazebo` CLI refuses `-g` there for this reason (gazebosim/gz-sim#44).
    # Use RViz for visualisation instead.
    gazebo_client_supported = platform.system() != "Darwin"
    gazebo_client = ExecuteProcess(
        cmd=[
            "ruby", ign_executable, "gazebo", "-g",
            "--force-version", "6",
        ],
        output="screen",
        condition=UnlessCondition(headless),
    ) if gazebo_client_supported else LogInfo(
        msg="macOS: skipping the Gazebo GUI (unsupported); use RViz to view the robot")

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
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--service-call-timeout", "60",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    # Stands in for joint_state_broadcaster when ros2_control is disabled.
    # Gazebo's JointStatePublisher has no rate control and fires every sim step,
    # so the raw topic arrives near 1 kHz; throttle it to the same 30 Hz
    # joint_state_broadcaster uses, which also paces /odom and /tf downstream.
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="joint_state_bridge",
        arguments=["/joint_states_gz@sensor_msgs/msg/JointState[ignition.msgs.Model"],
        remappings=[("/joint_states_gz", "/joint_states_raw")],
        output="screen",
        condition=UnlessCondition(use_ros2_control),
    )

    joint_state_throttle = Node(
        package="topic_tools",
        executable="throttle",
        name="joint_state_throttle",
        arguments=["messages", "/joint_states_raw", "30.0", "/joint_states"],
        output="screen",
        condition=UnlessCondition(use_ros2_control),
    )

    # Native MecanumDrive has no command timeout in Fortress 6.16, so keep the
    # public /cmd_vel contract and publish zero to the internal bridge topic when
    # commands stop. motion_bias_file adds the per-direction drift that makes an
    # uncalibrated mecanum base behave like the real one; it is resampled on
    # every launch, so tests asserting an exact trajectory must switch it off.
    # An empty path is how the watchdog is told to relay unmodified, so the two
    # variants differ only in that parameter.
    def _watchdog(bias_file, condition):
        return Node(
            package="yahboom_rosmaster_gazebo",
            executable="cmd_vel_watchdog.py",
            output="screen",
            parameters=[{"motion_bias_file": bias_file}],
            condition=condition,
        )

    # Ideal odometry from the raw /cmd_vel, before any drift is injected, so a
    # student can see estimated against commanded.
    calculated_odometry_node = Node(
        package="yahboom_rosmaster_gazebo",
        executable="calculated_odometry.py",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_rate": 50.0,
            "odom_frame_id": "calc_odom",
            "base_frame_id": "calc_base",
        }],
    )

    # A runtime node, not the contract probe. The probe validates and exits, so
    # launching it here left a process that always exits non-zero and broke
    # test_all_processes_exit_cleanly in every launch test including this file.
    ground_truth_tf_node = Node(
        package="yahboom_rosmaster_gazebo",
        executable="ground_truth_tf.py",
        output="screen",
    )

    motion_bias = LaunchConfiguration("motion_bias")
    cmd_vel_watchdog = _watchdog(motion_bias_config, IfCondition(motion_bias))
    cmd_vel_watchdog_unbiased = _watchdog("", UnlessCondition(motion_bias))

    # Encoder-style odometry from wheel joint states remains separate from the
    # measurement-only /ground_truth/odom bridge and owns odom->base TF.
    wheel_state_odometry = ExecuteProcess(
        cmd=["python3", wheel_odometry_script],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_rviz,
        declare_headless,
        declare_render_sensors,
        declare_use_ros2_control,
        declare_motion_bias,
        declare_motion_profile,
        # Force X11/XWayland for Gazebo GUI — prevents white window on Wayland + AMD GPU.
        # macOS has no xcb platform plugin; setting it there breaks every Qt app,
        # RViz included.
        *([] if platform.system() == "Darwin"
          else [SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb")]),
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
        OpaqueFunction(
            function=_launch_robot,
            args=[default_xacro, motion_profile_config],
        ),
        TimerAction(period=5.0, actions=[
            ros_gz_bridge,
            ros_gz_image_bridge,
            joint_state_bridge,
            joint_state_throttle,
            cmd_vel_watchdog,
            cmd_vel_watchdog_unbiased,
            calculated_odometry_node,
            ground_truth_tf_node,
        ]),
        TimerAction(period=12.0, actions=[
            load_joint_state_broadcaster,
            wheel_state_odometry,
        ]),
        OpaqueFunction(function=_launch_rviz),
    ])
