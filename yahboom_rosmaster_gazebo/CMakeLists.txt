cmake_minimum_required(VERSION 3.8)
project(yahboom_rosmaster_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Copy necessary files to designated locations in the project
install(
  DIRECTORY config launch models rviz scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/base_feedback_probe.py
    scripts/depth_geometry_probe.py
    scripts/imu_motion_probe.py
    scripts/lidar_geometry_probe.py
    scripts/sensor_contract_probe.py
    scripts/wheel_odometry_resilience_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_launch_test(
    test/sensor_contract.launch.py
    TARGET sensor_contract_empty
    TIMEOUT 90
    ARGS "world:=empty.world"
  )
  add_launch_test(
    test/sensor_contract.launch.py
    TARGET sensor_contract_cafe
    TIMEOUT 90
    ARGS "world:=cafe.world"
  )
  add_launch_test(
    test/base_feedback.launch.py
    TARGET base_feedback
    TIMEOUT 90
  )
  add_launch_test(
    test/depth_geometry.launch.py
    TARGET depth_geometry
    TIMEOUT 90
  )
  add_launch_test(
    test/lidar_geometry.launch.py
    TARGET lidar_geometry
    TIMEOUT 90
  )
  add_launch_test(
    test/imu_motion.launch.py
    TARGET imu_motion
    TIMEOUT 90
  )
  add_launch_test(
    test/wheel_odometry_resilience.launch.py
    TARGET wheel_odometry_resilience
    TIMEOUT 40
  )
endif()

ament_package()
