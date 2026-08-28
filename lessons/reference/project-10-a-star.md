# Project 10 Reference — A* Planning

Use these incomplete pieces with the Project 10 instructions.

## Wrapper Parameters

```cpp
const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
```

## Queue Scaffold

```cpp
auto heuristic = [&](int col, int row) {
  const double dc = static_cast<double>(col - goal_col);
  const double dr = static_cast<double>(row - goal_row);
  return std::hypot(dc, dr) * grid.resolution;
};

struct Entry
{
  double f;
  int col;
  int row;
};

auto cmp = [](const Entry & a, const Entry & b) {return a.f > b.f;};
std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> open(cmp);

open.push({heuristic(start_col, start_row), start_col, start_row});
```

## Search Loop Scaffold

```cpp
while (!open.empty()) {
  const Entry current = open.top();
  open.pop();

  const std::size_t current_index = grid.index(current.col, current.row);
  if (closed[current_index]) {
    continue;
  }
  closed[current_index] = true;

  if (current_index == goal_index) {
    // TODO: reconstruct and return the path
  }

  for (int neighbor = 0; neighbor < 8; ++neighbor) {
    // TODO: coordinates, validity checks, move cost, and relaxation
  }
}
```

## Relaxation Shape

```cpp
if (tentative_g < g_score[neighbor_index]) {
  g_score[neighbor_index] = tentative_g;
  came_from[neighbor_index] = static_cast<int>(current_index);
  open.push({tentative_g + heuristic(neighbor_col, neighbor_row),
             neighbor_col, neighbor_row});
}
```

## Reconstruction Shape

```cpp
std::vector<int> reversed;
int index = static_cast<int>(goal_index);

while (index != -1) {
  reversed.push_back(index);
  index = came_from[static_cast<std::size_t>(index)];
}

return std::vector<int>(reversed.rbegin(), reversed.rend());
```

## `package.xml`

The package already contains this file. Use it unchanged:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>a_star_planner</name>
  <version>0.1.0</version>
  <description>A* grid path planner component for the RoboNav software training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
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
```
