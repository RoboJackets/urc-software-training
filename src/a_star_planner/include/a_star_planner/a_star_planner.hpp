// ROS node wrapping the A* search core: caches /map, plans from the robot's TF
// pose to a /goal_pose, and publishes the result on /plan.

#pragma once

#include "robonav_training_common/occupancy_grid.hpp"

#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace robonav_training
{

class AStarPlanner : public rclcpp::Node
{
public:
  explicit AStarPlanner(const rclcpp::NodeOptions & options);

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
  void publishPath(const std::vector<int> & cell_path) const;

  // Parameters.
  int occupied_threshold_{50};
  double inflation_radius_{0.55};
  bool allow_unknown_{false};

  // Cached map.
  bool have_map_{false};
  GridMap grid_;
  std::vector<std::uint8_t> blocked_;     // nonzero = blocked (includes inflation)

  // ROS interfaces.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace robonav_training
