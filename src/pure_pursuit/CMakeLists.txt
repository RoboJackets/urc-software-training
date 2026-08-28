cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robonav_training_common REQUIRED)

include_directories(include)

# ROS-free pure pursuit geometry core.
add_library(pure_pursuit_core STATIC src/pure_pursuit_core.cpp)
set_target_properties(pure_pursuit_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(pure_pursuit_core robonav_training_common)

# Composable component (+ standalone `pure_pursuit` executable).
add_library(pure_pursuit_component SHARED src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit_component pure_pursuit_core)
ament_target_dependencies(pure_pursuit_component
  rclcpp rclcpp_components nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros robonav_training_common)
rclcpp_components_register_node(pure_pursuit_component
  PLUGIN "robonav_training::PurePursuit"
  EXECUTABLE pure_pursuit)

install(TARGETS pure_pursuit_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
