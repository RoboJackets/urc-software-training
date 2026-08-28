// pure_pursuit.hpp
// ROS node wrapping the Pure Pursuit geometry: reads parameters and the robot
// pose from TF, subscribes to the planned path, and publishes velocity
// commands. The path-tracking math lives in pure_pursuit_core.hpp.

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robonav_training
{

class PurePursuit : public rclcpp::Node
{
public:
  explicit PurePursuit(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishStop();
  void controlStep();

  // Parameters
  double lookahead_distance_;
  double desired_linear_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  double control_rate_;

  // State
  nav_msgs::msg::Path path_;

  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace robonav_training
