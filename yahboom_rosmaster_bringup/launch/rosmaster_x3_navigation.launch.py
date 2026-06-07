#!/usr/bin/env python3
"""
Launch Nav2 for the Yahboom ROSMASTER X3 robot in Gazebo.

This launch file sets up a complete ROS 2 navigation environment.

:author: Addison Sears-Collins
:date: December 10, 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the Nav2 launch description."""
    package_name_docking = 'yahboom_rosmaster_docking'
    package_name_gazebo = 'yahboom_rosmaster_gazebo'
    package_name_localization = 'yahboom_rosmaster_localization'
    package_name_navigation = 'yahboom_rosmaster_navigation'

    apriltag_launch_file_path = 'launch/apriltag_dock_pose_publisher.launch.py'
    gazebo_launch_file_path = 'launch/rosmaster_gazebo_fortress.launch.py'
    ekf_launch_file_path = 'launch/ekf_gazebo.launch.py'
    ekf_config_file_path = 'config/ekf.yaml'
    map_file_path = 'maps/cafe_world_map.yaml'
    nav2_params_path = 'config/rosmaster_x3_nav2_default_params.yaml'

    pkg_share_docking = FindPackageShare(
        package=package_name_docking).find(package_name_docking)
    pkg_share_gazebo = FindPackageShare(
        package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_localization = FindPackageShare(
        package=package_name_localization).find(package_name_localization)
    pkg_share_navigation = FindPackageShare(
        package=package_name_navigation).find(package_name_navigation)

    default_apriltag_launch_path = PathJoinSubstitution(
        [pkg_share_docking, apriltag_launch_file_path])
    default_gazebo_launch_path = PathJoinSubstitution(
        [pkg_share_gazebo, gazebo_launch_file_path])
    default_world_path = PathJoinSubstitution(
        [pkg_share_gazebo, 'worlds', 'cafe.world'])
    default_ekf_launch_path = PathJoinSubstitution(
        [pkg_share_localization, ekf_launch_file_path])
    default_ekf_config_path = PathJoinSubstitution(
        [pkg_share_localization, ekf_config_file_path])

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = PathJoinSubstitution([nav2_dir, 'launch'])
    nav2_params_file_path = PathJoinSubstitution([pkg_share_navigation, nav2_params_path])
    static_map_path = PathJoinSubstitution([pkg_share_navigation, map_file_path])

    autostart = LaunchConfiguration('autostart')
    camera_namespace = LaunchConfiguration('camera_namespace')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    ekf_launch_file = LaunchConfiguration('ekf_launch_file')
    gazebo_launch_file = LaunchConfiguration('gazebo_launch_file')
    map_yaml_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam = LaunchConfiguration('slam')
    use_composition = LaunchConfiguration('use_composition')
    use_namespace = LaunchConfiguration('use_namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack')

    declare_camera_namespace_cmd = DeclareLaunchArgument(
        name='camera_namespace',
        default_value='cam_1',
        description='Namespace for the camera')

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF configuration YAML file')

    declare_ekf_launch_file_cmd = DeclareLaunchArgument(
        name='ekf_launch_file',
        default_value=default_ekf_launch_path,
        description='Full path to the EKF launch file to use')

    declare_gazebo_launch_file_cmd = DeclareLaunchArgument(
        name='gazebo_launch_file',
        default_value=default_gazebo_launch_path,
        description='Full path to the Gazebo launch file to use')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to the Gazebo world file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        name='nav2_params_file',
        default_value=nav2_params_file_path,
        description='Full path to the ROS2 parameters file to use for navigation nodes')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run Gazebo server without the GUI client')

    declare_use_composition_cmd = DeclareLaunchArgument(
        name='use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        name='use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Launch the simulator RViz view')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    start_apriltag_dock_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([default_apriltag_launch_path]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_namespace': camera_namespace
        }.items()
    )

    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ekf_launch_file]),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'world': world,
            'rviz': rviz,
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )

    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_launch_dir, 'bringup_launch.py'])]),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_camera_namespace_cmd)
    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_ekf_launch_file_cmd)
    ld.add_action(declare_gazebo_launch_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_apriltag_dock_cmd)
    ld.add_action(start_ekf_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    return ld
