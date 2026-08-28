# Project 11 Reference — Pure Pursuit

Use these incomplete pieces with the Project 11 instructions.

## Wrapper Parameters

```cpp
const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
const auto cmd_vel_topic =
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
```

## Lookahead Scaffold

```cpp
std::size_t lookahead_index = path.size() - 1;

for (std::size_t i = nearest_index; i < path.size(); ++i) {
  const double distance =
    std::hypot(path[i][0] - robot_x, path[i][1] - robot_y);
  if (distance >= lookahead_distance) {
    lookahead_index = i;
    break;
  }
}
```

## World-To-Robot Transform

```cpp
const double dx = path[lookahead_index][0] - robot_x;
const double dy = path[lookahead_index][1] - robot_y;

const double x_r =
  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
const double y_r =
  -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
```

## Command Shape

```cpp
PursuitCommand command{0.0, 0.0, false};

if (x_r < 0.0) {
  // TODO: slow forward command and turn toward the point
} else {
  const double distance = std::hypot(x_r, y_r);
  const double curvature =
    (distance > 1e-6) ? (2.0 * y_r) / (distance * distance) : 0.0;

  // TODO: apply goal slowdown
  // TODO: set linear and clamped angular command
}

return command;
```

Use `normalizeAngle(std::atan2(y_r, x_r))` when choosing the turn direction for
a point behind the robot.

## `package.xml`

The package already contains this file. Use it unchanged:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pure_pursuit</name>
  <version>0.1.0</version>
  <description>Pure pursuit path-following controller component for the RoboNav software training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>robonav_training_common</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## `CMakeLists.txt`

The package already contains this file. Use it unchanged:

```cmake
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
```
