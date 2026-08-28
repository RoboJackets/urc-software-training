cmake_minimum_required(VERSION 3.8)
project(a_star_planner)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robonav_training_common REQUIRED)

include_directories(include)

# ROS-free A* search core.
add_library(a_star_search STATIC src/a_star_search.cpp)
set_target_properties(a_star_search PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(a_star_search robonav_training_common)

# Composable component (+ standalone `a_star_planner` executable).
add_library(a_star_planner_component SHARED src/a_star_planner.cpp)
target_link_libraries(a_star_planner_component a_star_search)
ament_target_dependencies(a_star_planner_component
  rclcpp rclcpp_components nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs robonav_training_common)
rclcpp_components_register_node(a_star_planner_component
  PLUGIN "robonav_training::AStarPlanner"
  EXECUTABLE a_star_planner)

install(TARGETS a_star_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
