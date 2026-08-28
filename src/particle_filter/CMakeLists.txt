cmake_minimum_required(VERSION 3.8)
project(particle_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Composable component (+ standalone `particle_filter` executable).
add_library(particle_filter_component SHARED
  src/particle_filter.cpp
  src/pf_compute_score.cpp
  src/pf_motion_update.cpp
  src/pf_random_helpers.cpp
  src/pf_resample.cpp
  src/pf_tf_helpers.cpp)
ament_target_dependencies(particle_filter_component
  rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(particle_filter_component
  PLUGIN "particle_filter::ParticleFilter"
  EXECUTABLE particle_filter)

install(TARGETS particle_filter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
