# Project 0 Reference — NumberScaler

Use these incomplete pieces with the Project 0 instructions.

## Header Scaffold

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace robonav_training
{

class NumberScaler : public rclcpp::Node
{
public:
  explicit NumberScaler(const rclcpp::NodeOptions & options);

private:
  // TODO: callback, scale, publisher, and subscription
};

}  // namespace robonav_training
```

Member type shapes:

```cpp
void onNumber(const std_msgs::msg::Float64::SharedPtr msg);
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_sub_;
```

## Source Scaffold

```cpp
#include "ros_cpp_practice/number_scaler.hpp"

#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace robonav_training
{

NumberScaler::NumberScaler(const rclcpp::NodeOptions & options)
: Node("number_scaler", options)
{
  // TODO: parameter, publisher, and subscription
}

// Define onNumber(...) here.

}  // namespace robonav_training

RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::NumberScaler)
```

Constructor shapes:

```cpp
scale_ = declare_parameter<double>("scale", 2.0);
output_pub_ = create_publisher<std_msgs::msg::Float64>("/practice/output", 10);
input_sub_ = create_subscription<std_msgs::msg::Float64>(
  "/practice/input", 10,
  std::bind(&NumberScaler::onNumber, this, std::placeholders::_1));
```

## Callback Shape

```cpp
void NumberScaler::onNumber(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  std_msgs::msg::Float64 output;
  // TODO: calculate output.data and publish
}
```

## `package.xml`

Copy this complete file to `ros_cpp_practice/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros_cpp_practice</name>
  <version>0.1.0</version>
  <description>A minimal ROS 2 C++ publisher/subscriber exercise for RoboNav training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>std_msgs</depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## `CMakeLists.txt`

Copy this complete file to `ros_cpp_practice/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ros_cpp_practice)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(number_scaler_component SHARED src/number_scaler.cpp)
target_include_directories(number_scaler_component PUBLIC include)
ament_target_dependencies(number_scaler_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(number_scaler_component
  PLUGIN "robonav_training::NumberScaler"
  EXECUTABLE number_scaler)

install(TARGETS number_scaler_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

## Launch Scaffold

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_cpp_practice",
                executable="number_scaler",
                name="number_scaler",
                # TODO: pass scale=2.0
            )
        ]
    )
```

Parameter shape:

```python
parameters=[{"scale": 2.0}],
```
