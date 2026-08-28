# C++ Node Patterns

This page shows the C++ syntax patterns used by the training packages.

## Header File Skeleton

Use the header for class declarations, callback declarations, state, publishers,
subscribers, timers, and TF objects.

```cpp
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robonav_training
{

class ExampleNode : public rclcpp::Node
{
public:
  explicit ExampleNode(const rclcpp::NodeOptions & options);

private:
  void onMessage(const std_msgs::msg::String::SharedPtr msg);
  void onTimer();

  std::string input_topic_;
  double publish_rate_{10.0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robonav_training
```

Answer-key or later-starter examples:

- `src/wheel_odometry/include/wheel_odometry/wheel_odometry.hpp`
- `src/a_star_planner/include/a_star_planner/a_star_planner.hpp`
- `src/pure_pursuit/include/pure_pursuit/pure_pursuit.hpp`

## Source File Skeleton

Use the `.cpp` for constructor logic and callback bodies.

```cpp
#include "example_package/example_node.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace robonav_training
{

ExampleNode::ExampleNode(const rclcpp::NodeOptions & options)
: Node("example_node", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/input");
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

  sub_ = create_subscription<std_msgs::msg::String>(
    input_topic_, 10,
    std::bind(&ExampleNode::onMessage, this, std::placeholders::_1));

  pub_ = create_publisher<std_msgs::msg::String>("/output", 10);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&ExampleNode::onTimer, this));
}

void ExampleNode::onMessage(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Heard: %s", msg->data.c_str());
}

void ExampleNode::onTimer()
{
  std_msgs::msg::String msg;
  msg.data = "hello";
  pub_->publish(msg);
}

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::ExampleNode)
```

Key details:

- `declare_parameter<T>("name", default_value)` registers and reads a parameter.
- `create_subscription<MessageType>(topic, qos, callback)` creates a subscriber.
- `create_publisher<MessageType>(topic, qos)` creates a publisher.
- `std::bind(..., std::placeholders::_1)` connects a one-argument callback.
- `RCLCPP_COMPONENTS_REGISTER_NODE(...)` makes the class loadable as a component.

## Publisher Pattern

```cpp
#include <geometry_msgs/msg/twist.hpp>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

geometry_msgs::msg::Twist cmd;
cmd.linear.x = 0.3;
cmd.angular.z = 0.0;
cmd_vel_pub_->publish(cmd);
```

Answer-key or later-starter example:

- `src/pure_pursuit/src/pure_pursuit.cpp`

## Subscriber Pattern

```cpp
#include <sensor_msgs/msg/joint_state.hpp>

void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", 10,
  std::bind(&MyNode::onJointState, this, std::placeholders::_1));
```

Callback:

```cpp
void MyNode::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.empty() || msg->position.empty()) {
    return;
  }
}
```

Implemented in Lesson 5:

- `src/wheel_odometry/src/wheel_odometry.cpp`

## Parameter Pattern

```cpp
wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
left_joint_name_ = declare_parameter<std::string>("left_joint_name", "left_wheel_joint");
allow_unknown_ = declare_parameter<bool>("allow_unknown", false);
```

Parameters can be set from launch:

```python
parameters=[
    {
        "use_sim_time": use_sim_time,
        "wheel_radius": 0.075,
    }
]
```

Inspect at runtime after the node exists and is running:

```sh
ros2 param list /wheel_odometry
ros2 param get /wheel_odometry wheel_radius
```

## Timer Pattern

Use timers for repeated control loops, not for sensor callbacks.

```cpp
control_rate_ = declare_parameter<double>("control_rate", 20.0);

timer_ = create_wall_timer(
  std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / control_rate_)),
  std::bind(&PurePursuit::controlStep, this));
```

Answer-key or later-starter example:

- `src/pure_pursuit`

## TF Listener Pattern

Use a TF listener when your node needs the current robot pose or a sensor offset.

Header:

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
```

Constructor:

```cpp
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

Lookup:

```cpp
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

geometry_msgs::msg::TransformStamped tf;
try {
  tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
} catch (const tf2::TransformException & ex) {
  RCLCPP_WARN(get_logger(), "Could not get TF: %s", ex.what());
  return;
}

const double x = tf.transform.translation.x;
const double y = tf.transform.translation.y;
```

Answer-key or later-starter examples:

- `src/a_star_planner`
- `src/pure_pursuit`
- `src/particle_filter`

## TF Broadcaster Pattern

Use a TF broadcaster when your node owns a transform.

```cpp
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

geometry_msgs::msg::TransformStamped tf_msg;
tf_msg.header.stamp = now();
tf_msg.header.frame_id = "map";
tf_msg.child_frame_id = "odom";
tf_msg.transform.translation.x = x;
tf_msg.transform.translation.y = y;
tf_msg.transform.translation.z = 0.0;

tf2::Quaternion q;
q.setRPY(0.0, 0.0, yaw);
tf_msg.transform.rotation = tf2::toMsg(q);

tf_broadcaster_->sendTransform(tf_msg);
```

Answer-key or later-starter example:

- `src/particle_filter/src/pf_tf_helpers.cpp`

## ROS-Free Core Pattern

Keep math and algorithms separate from ROS when possible.

Header:

```cpp
#pragma once

#include <vector>

namespace robonav_training
{

std::vector<int> astarSearch(/* plain C++ inputs */);

}  // namespace robonav_training
```

Implementation:

```cpp
#include "a_star_planner/a_star_search.hpp"

namespace robonav_training
{

std::vector<int> astarSearch(/* plain C++ inputs */)
{
  // No rclcpp, no topics, no TF.
}

}  // namespace robonav_training
```

Why this matters:

- Easier to unit test.
- Easier to read.
- You can debug algorithm logic without launching ROS.

Answer-key or later-starter examples:

- `src/a_star_planner/src/a_star_search.cpp`
- `src/pure_pursuit/src/pure_pursuit_core.cpp`
- `src/map_server/src/map_loader.cpp`
