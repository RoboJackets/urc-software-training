# Project 9 Reference — Particle Filter Motion Update

Use this incomplete function with the Project 9 instructions.

## Motion-Update Scaffold

```cpp
void ParticleFilter::applyMotionUpdate(
  double dx_body, double dy_body, double dyaw,
  double translation, double rotation)
{
  // TODO: calculate sigma_x, sigma_y, and sigma_theta

  for (auto & particle : particles_) {
    // TODO: sample noisy_dx, noisy_dy, and noisy_dyaw

    const double c = std::cos(particle.theta);
    const double s = std::sin(particle.theta);
    const double world_dx = c * noisy_dx - s * noisy_dy;
    const double world_dy = s * noisy_dx + c * noisy_dy;

    particle.x += world_dx;
    particle.y += world_dy;
    particle.theta = wrapAngle(particle.theta + noisy_dyaw);
  }
}
```

Call `gaussianNoise(...)` separately for each particle. Do not change weights.

## `package.xml`

The package already contains this file. Use it unchanged:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>particle_filter</name>
  <version>0.1.0</version>
  <description>Lidar Monte Carlo Localization (particle filter) component for the RoboNav software training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

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
```
