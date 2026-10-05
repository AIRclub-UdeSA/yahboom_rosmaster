# Standalone reproducer for the image_bridge shutdown crash (#55). Build out of
# tree with ROS sourced:  cmake -S . -B /tmp/ibr && cmake --build /tmp/ibr
cmake_minimum_required(VERSION 3.8)
project(image_bridge_repro CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

foreach(variant orig guard)
  add_executable(image_bridge_${variant} image_bridge_variants.cpp)
  ament_target_dependencies(image_bridge_${variant} rclcpp image_transport ros_gz_bridge)
  target_link_libraries(image_bridge_${variant} ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)
endforeach()
target_compile_definitions(image_bridge_guard PRIVATE GUARD_CALLBACK=1)

add_executable(gz_image_publisher gz_image_publisher.cpp)
target_link_libraries(gz_image_publisher ignition-transport11::ignition-transport11 ignition-msgs8::ignition-msgs8)
