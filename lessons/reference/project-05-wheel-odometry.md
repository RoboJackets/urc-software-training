# Project 5 Reference — Wheel Odometry

Use these incomplete pieces with the Project 5 instructions.

## Node Scaffolds

Header:

```cpp
#pragma once

#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robonav_training
{

class WheelOdometry : public rclcpp::Node
{
public:
  explicit WheelOdometry(const rclcpp::NodeOptions & options);

private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishOdometry(const rclcpp::Time & stamp, double vx, double wz);

  // TODO: parameters, pose, previous sample, publisher, and subscription
};

}  // namespace robonav_training
```

Member type shapes:

```cpp
double wheel_radius_;
std::string left_joint_name_;
rclcpp::Time stamp_prev_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
```

Use `double` for all numeric state and `std::string` for both joint names.

Source shape:

```cpp
#include "wheel_odometry/wheel_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robonav_training_common/angles.hpp"

namespace robonav_training
{

WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options)
: Node("wheel_odometry", options)
{
  // TODO: parameters, publisher, subscription
}

// Define onJointState(...) and publishOdometry(...) here.

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::WheelOdometry)
```

## Constructor Shapes

```cpp
wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
wheel_separation_ = declare_parameter<double>("wheel_separation", 0.34);

odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", 10,
  std::bind(&WheelOdometry::onJointState, this, std::placeholders::_1));
```

Declare the two joint-name parameters the same way with type `std::string`.

## Method Shapes

```cpp
void WheelOdometry::onJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // TODO: read the wheels, update the pose, and publish
}

void WheelOdometry::publishOdometry(
  const rclcpp::Time & stamp, double vx, double wz)
{
  nav_msgs::msg::Odometry odom;
  // TODO: fill and publish odom
}
```

## Joint Lookup Helper

```cpp
static int findJointIndex(
  const sensor_msgs::msg::JointState & msg, const std::string & name)
{
  for (std::size_t i = 0; i < msg.name.size(); ++i) {
    if (msg.name[i] == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}
```

After lookup, verify the larger index is smaller than `msg->position.size()`
before reading either position.

## Odometry Message Lines

```cpp
tf2::Quaternion q;
q.setRPY(0.0, 0.0, theta_);
odom.pose.pose.orientation = tf2::toMsg(q);

odom.twist.twist.linear.x = vx;
odom.twist.twist.angular.z = wz;

odom.pose.covariance[0] = 1e-3;
odom.pose.covariance[7] = 1e-3;
odom.pose.covariance[35] = 1e-2;
odom.twist.covariance[0] = 1e-3;
odom.twist.covariance[35] = 1e-2;

odom_pub_->publish(odom);
```

## `package.xml`

Copy this complete file to `wheel_odometry/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wheel_odometry</name>
  <version>0.1.0</version>
  <description>Differential-drive wheel odometry component for the RoboNav software training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
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

Copy this complete file to `wheel_odometry/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(robonav_training_common REQUIRED)

include_directories(include)

# A single composable component. The wheel-odometry math is short enough to live
# right in the node, so there is no separate algorithm-core library here.
# rclcpp_components_register_node ALSO emits a standalone `wheel_odometry`
# executable for `ros2 run wheel_odometry wheel_odometry`.
add_library(wheel_odometry_component SHARED src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry_component
  rclcpp rclcpp_components nav_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs robonav_training_common)
rclcpp_components_register_node(wheel_odometry_component
  PLUGIN "robonav_training::WheelOdometry"
  EXECUTABLE wheel_odometry)

install(TARGETS wheel_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

Keep the `find_package(...)` lines generated by `ros2 pkg create`.

## Launch Scaffold

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wheel_odometry",
                executable="wheel_odometry",
                name="wheel_odometry",
                output="screen",
                # TODO: pass use_sim_time=True
            )
        ]
    )
```

Parameter shape:

```python
parameters=[{"use_sim_time": True}],
```
