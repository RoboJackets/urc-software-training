# CMake And package.xml

When you use a ROS dependency in C++, you must usually update both:

- `CMakeLists.txt` so the code can build and link.
- `package.xml` so ROS knows the package depends on it.

## Common Dependencies In This Repo

| Dependency | Needed for |
| --- | --- |
| `rclcpp` | C++ ROS nodes |
| `rclcpp_components` | composable C++ nodes |
| `std_msgs` | basic messages such as strings, numbers, bools |
| `geometry_msgs` | `Twist`, `PoseStamped`, `PoseArray`, transforms |
| `nav_msgs` | `Odometry`, `Path`, `OccupancyGrid` |
| `sensor_msgs` | `JointState`, `LaserScan`, `Imu` |
| `tf2` | transform math, quaternions, yaw helpers |
| `tf2_ros` | TF listener and broadcaster |
| `tf2_geometry_msgs` | conversions between TF and geometry messages |
| `launch_ros` | Python ROS launch files |

## package.xml Dependency Syntax

For dependencies used to build and run C++ nodes:

```xml
<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

For launch-only dependencies:

```xml
<exec_depend>launch_ros</exec_depend>
```

If a package only provides launch/config files, it may have mostly
`<exec_depend>` entries.

## CMake Package Skeleton

Start with the package only. Add dependencies, targets, registration, and install
rules from the separate entries below as the package needs them.

```cmake
cmake_minimum_required(VERSION 3.8)
project(example_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_package()
```

## Adding A Component Library

Add only the dependencies used by the node. Place this before `ament_package()`:

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(example_component SHARED src/example_node.cpp)
target_include_directories(example_component PUBLIC include)
ament_target_dependencies(example_component
  rclcpp rclcpp_components std_msgs)

install(TARGETS example_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
```

Component registration and optional folder installs are separate steps below.

## Adding A ROS-Free Core Library

Use this when the algorithm can be tested without ROS.

```cmake
add_library(example_core STATIC src/example_core.cpp)
set_target_properties(example_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(example_core PUBLIC include)

add_library(example_component SHARED src/example_node.cpp)
target_link_libraries(example_component example_core)
ament_target_dependencies(example_component
  rclcpp rclcpp_components geometry_msgs)
```

Answer-key or later-starter examples:

- `src/a_star_planner/CMakeLists.txt`
- `src/pure_pursuit/CMakeLists.txt`
- `src/map_server/CMakeLists.txt`

## Registering A Component

C++ file:

```cpp
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::ExampleNode)
```

CMake:

```cmake
rclcpp_components_register_node(example_component
  PLUGIN "robonav_training::ExampleNode"
  EXECUTABLE example_node)
```

This creates two ways to run the node:

- as a component in a container
- as a standalone executable with `ros2 run example_package example_node`

## Installing Launch, Config, Maps, URDF

Install folders that need to be available through package share:

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
```

If a launch file uses `FindPackageShare("your_package")`, the target file must be
installed under `share/${PROJECT_NAME}`.

## Add A New Message Dependency Checklist

Example: you start using `sensor_msgs::msg::LaserScan`.

1. Include it in C++:

   ```cpp
   #include <sensor_msgs/msg/laser_scan.hpp>
   ```

2. Add it to `package.xml`:

   ```xml
   <depend>sensor_msgs</depend>
   ```

3. Add it to `CMakeLists.txt`:

   ```cmake
   find_package(sensor_msgs REQUIRED)
   ament_target_dependencies(your_target sensor_msgs)
   ```

4. Rebuild and source:

   ```sh
   colcon build --symlink-install
   source install/setup.bash
   ```

## Common Build Errors

`fatal error: nav_msgs/msg/path.hpp: No such file or directory`

- Missing `find_package(nav_msgs REQUIRED)` or missing `ament_target_dependencies`.

`package 'my_package' not found`

- Forgot to build, source, or install package files.
