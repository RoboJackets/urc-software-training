// pure_pursuit.cpp
// ROS wrapper for the Pure Pursuit follower: parameters, TF pose lookup, the
// /plan subscription and /cmd_vel publisher. Geometry lives in the core.

#include "pure_pursuit/pure_pursuit.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "pure_pursuit/pure_pursuit_core.hpp"

namespace robonav_training
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

PurePursuit::PurePursuit(const rclcpp::NodeOptions & options)
: Node("pure_pursuit", options)
{
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.3);
  desired_linear_speed_ =
    declare_parameter<double>("desired_linear_speed", 0.3);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.5);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.2);
  control_rate_ = declare_parameter<double>("control_rate", 20.0);
  // TODO(Lesson 11): declare plan_topic and cmd_vel_topic as string parameters
  // with these course defaults.
  const std::string plan_topic = "/plan";
  const std::string cmd_vel_topic = "/cmd_vel";

  if (!std::isfinite(lookahead_distance_) || lookahead_distance_ <= 0.0) {
    throw std::invalid_argument("lookahead_distance must be finite and greater than zero");
  }
  if (!std::isfinite(desired_linear_speed_) || desired_linear_speed_ < 0.0) {
    throw std::invalid_argument("desired_linear_speed must be finite and non-negative");
  }
  if (!std::isfinite(max_angular_speed_) || max_angular_speed_ < 0.0) {
    throw std::invalid_argument("max_angular_speed must be finite and non-negative");
  }
  if (!std::isfinite(goal_tolerance_) || goal_tolerance_ < 0.0) {
    throw std::invalid_argument("goal_tolerance must be finite and non-negative");
  }
  if (!std::isfinite(control_rate_) || control_rate_ <= 0.0) {
    throw std::invalid_argument("control_rate must be finite and greater than zero");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    plan_topic, 10,
    std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PurePursuit::controlStep, this));

  RCLCPP_INFO(
    get_logger(),
    "PurePursuit ready (lookahead=%.2f m, speed=%.2f m/s).",
    lookahead_distance_, desired_linear_speed_);
}

void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = *msg;
  if (path_.poses.empty()) {
    publishStop();
  }
}

void PurePursuit::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

void PurePursuit::controlStep()
{
  // No path: stay silent so manual teleop can own /cmd_vel.
  if (path_.poses.empty()) {
    return;
  }

  // Robot pose from the map -> base_footprint transform.
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      "map", "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF map->base_footprint unavailable: %s", ex.what());
    publishStop();
    return;
  }
  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double ryaw = yawFromQuaternion(tf.transform.rotation);

  // Convert the Path poses into the plain {x, y} polyline the core expects.
  std::vector<std::array<double, 2>> path;
  path.reserve(path_.poses.size());
  for (const auto & pose : path_.poses) {
    path.push_back({pose.pose.position.x, pose.pose.position.y});
  }

  const PursuitCommand result = computePursuitCommand(
    rx, ry, ryaw, path, lookahead_distance_, desired_linear_speed_,
    max_angular_speed_, goal_tolerance_);

  // Goal reached: stop and forget the path so we don't re-trigger.
  if (result.goal_reached) {
    publishStop();
    path_.poses.clear();
    RCLCPP_INFO(get_logger(), "goal reached");
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = result.linear;
  cmd.angular.z = result.angular;
  cmd_vel_pub_->publish(cmd);
}

} // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::PurePursuit)
