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

## CMake Fragments

Add before `ament_package()`:

```cmake
set(CMAKE_CXX_STANDARD 17)

add_library(number_scaler_component SHARED src/number_scaler.cpp)
target_include_directories(number_scaler_component PUBLIC include)
ament_target_dependencies(number_scaler_component
  rclcpp rclcpp_components std_msgs)

rclcpp_components_register_node(number_scaler_component
  PLUGIN "robonav_training::NumberScaler"
  EXECUTABLE number_scaler)

install(TARGETS number_scaler_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
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
