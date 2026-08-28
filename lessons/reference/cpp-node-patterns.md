# C++ Node Patterns

This page shows the C++ syntax patterns used by the training packages.

## Modern C++ Smart Pointers

ROS 2 uses smart pointers to make object ownership and lifetime explicit. Brush
up on these before the larger C++ lessons:

- `std::unique_ptr<T>` means one owner. It cannot be copied, but ownership can be
  moved. Create one with `std::make_unique<T>(...)`.
- `std::shared_ptr<T>` means multiple pieces of code may share ownership. The
  object is destroyed after the last owner releases it. Create one with
  `std::make_shared<T>(...)`.
- ROS aliases such as `Message::SharedPtr` and
  `rclcpp::Publisher<Message>::SharedPtr` are shared pointers.
- Use `pointer->method()` to access an object through either kind of pointer.

Prefer `unique_ptr` when one object clearly owns a resource and `shared_ptr` when
an API, such as a ROS callback, intentionally shares it. The examples below use
both; you do not need to manage either one with `new` or `delete`.

## Header File Skeleton

Start with only the node class and constructor declaration. Add callbacks and
members later from the focused patterns below when the node actually needs them.

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace robonav_training
{

class ExampleNode : public rclcpp::Node
{
public:
  explicit ExampleNode(const rclcpp::NodeOptions & options);

private:
  // Add only the callbacks and members this node needs.
};

}  // namespace robonav_training
```

Answer-key or later-starter examples:

- `src/wheel_odometry/include/wheel_odometry/wheel_odometry.hpp`
- `src/a_star_planner/include/a_star_planner/a_star_planner.hpp`
- `src/pure_pursuit/include/pure_pursuit/pure_pursuit.hpp`

## Source File Skeleton

Start with the constructor and component registration. Add parameter declarations,
ROS interfaces, and callback bodies from the separate entries below.

```cpp
#include "example_package/example_node.hpp"

namespace robonav_training
{

ExampleNode::ExampleNode(const rclcpp::NodeOptions & options)
: Node("example_node", options)
{
  // Add only the setup this node needs.
}

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::ExampleNode)
```

`RCLCPP_COMPONENTS_REGISTER_NODE(...)` makes the class loadable as a component.
The focused entries below show what to add inside this scaffold.

## Publisher Pattern

Header additions:

```cpp
#include <geometry_msgs/msg/twist.hpp>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
```

Constructor addition:

```cpp
cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
```

Where the node publishes:

```cpp
geometry_msgs::msg::Twist cmd;
cmd.linear.x = 0.3;
cmd.angular.z = 0.0;
cmd_vel_pub_->publish(cmd);
```

Answer-key or later-starter example:

- `src/pure_pursuit/src/pure_pursuit.cpp`

## Subscriber Pattern

Header additions:

```cpp
#include <sensor_msgs/msg/joint_state.hpp>

void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
```

Source include and constructor addition:

```cpp
#include <functional>

joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
  "/joint_states", 10,
  std::bind(&MyNode::onJointState, this, std::placeholders::_1));
```

Source callback definition:

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

Header member:

```cpp
double wheel_radius_{0.075};
```

Constructor addition:

```cpp
wheel_radius_ = declare_parameter<double>("wheel_radius", 0.075);
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

Header additions:

```cpp
void controlStep();
double control_rate_{20.0};
rclcpp::TimerBase::SharedPtr timer_;
```

Source includes and constructor addition:

```cpp
#include <chrono>
#include <functional>

control_rate_ = declare_parameter<double>("control_rate", 20.0);

timer_ = create_wall_timer(
  std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / control_rate_)),
  std::bind(&PurePursuit::controlStep, this));
```

Define `controlStep()` separately in the source file.

Answer-key or later-starter example:

- `src/pure_pursuit`

## TF Listener Pattern

Use a TF listener when your node needs the current robot pose or a sensor offset.

Header:

```cpp
#include <memory>
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
