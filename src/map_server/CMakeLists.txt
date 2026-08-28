cmake_minimum_required(VERSION 3.8)
project(map_server)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# Map-file loader core (static). The one core needing ROS messages + yaml-cpp;
# yaml-cpp stays private to this translation unit.
add_library(map_loader STATIC src/map_loader.cpp)
set_target_properties(map_loader PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(map_loader nav_msgs tf2 tf2_geometry_msgs)
target_include_directories(map_loader PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(map_loader ${YAML_CPP_LIBRARIES})

# Composable component (+ standalone `map_server` executable).
add_library(map_server_component SHARED src/map_server.cpp)
target_link_libraries(map_server_component map_loader)
ament_target_dependencies(map_server_component
  rclcpp rclcpp_components nav_msgs)
rclcpp_components_register_node(map_server_component
  PLUGIN "robonav_training::MapServer"
  EXECUTABLE map_server)

install(TARGETS map_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch maps DESTINATION share/${PROJECT_NAME})

ament_package()
