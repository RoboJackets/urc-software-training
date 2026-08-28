// ROS node for the A* grid path planner: caches /map, plans from the robot's TF
// pose to a /goal_pose, and publishes the result on /plan.

#include "a_star_planner/a_star_planner.hpp"

#include "a_star_planner/a_star_search.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robonav_training
{

AStarPlanner::AStarPlanner(const rclcpp::NodeOptions & options)
: Node("a_star_planner", options)
{
  occupied_threshold_ = static_cast<int>(declare_parameter<int>("occupied_threshold", 50));
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.55);
  allow_unknown_ = declare_parameter<bool>("allow_unknown", false);
  // TODO(Lesson 10): declare map_topic, goal_topic, and plan_topic as string
  // parameters with these course defaults.
  const std::string map_topic = "/map";
  const std::string goal_topic = "/goal_pose";
  const std::string plan_topic = "/plan";

  if (occupied_threshold_ < 0 || occupied_threshold_ > 100) {
    throw std::invalid_argument("occupied_threshold must be between 0 and 100");
  }
  if (!std::isfinite(inflation_radius_) || inflation_radius_ < 0.0) {
    throw std::invalid_argument("inflation_radius must be finite and non-negative");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // The map is latched: transient_local QoS delivers the last map on connect.
  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, map_qos,
    std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, 10,
    std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic, 10);

  RCLCPP_INFO(
    get_logger(), "A* planner ready; waiting for %s and %s.",
    map_topic.c_str(), goal_topic.c_str());
}

// Cache the grid geometry and precompute the inflated obstacle mask once per map.
void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  grid_.width = static_cast<int>(msg->info.width);
  grid_.height = static_cast<int>(msg->info.height);
  grid_.resolution = msg->info.resolution;
  grid_.origin_x = msg->info.origin.position.x;
  grid_.origin_y = msg->info.origin.position.y;

  const std::size_t cell_count = grid_.cellCount();

  // Mark hard-blocked cells: occupied, or unknown when allow_unknown is false.
  std::vector<std::uint8_t> raw_blocked(cell_count, 0);
  for (std::size_t i = 0; i < cell_count && i < msg->data.size(); ++i) {
    const int8_t value = msg->data[i];
    const bool unknown = (value < 0);
    const bool occupied = (value >= occupied_threshold_);
    if (occupied || (unknown && !allow_unknown_)) {
      raw_blocked[i] = 1;
    }
  }

  // Inflate: grow each obstacle by inflation_radius (in cells) to keep the
  // robot body clear of walls.
  blocked_ = raw_blocked;
  const int radius_cells =
    (grid_.resolution > 0.0) ?
    static_cast<int>(std::ceil(inflation_radius_ / grid_.resolution)) :
    0;

  if (radius_cells > 0) {
    const double radius_sq =
      static_cast<double>(radius_cells) * static_cast<double>(radius_cells);
    for (int row = 0; row < grid_.height; ++row) {
      for (int col = 0; col < grid_.width; ++col) {
        if (raw_blocked[grid_.index(col, row)] == 0) {
          continue;
        }
        // Stamp a disk of blocked cells around this obstacle cell.
        for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
          for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
            if (static_cast<double>(dr * dr + dc * dc) > radius_sq) {
              continue;
            }
            const int nc = col + dc;
            const int nr = row + dr;
            if (grid_.inBounds(nc, nr)) {
              blocked_[grid_.index(nc, nr)] = 1;
            }
          }
        }
      }
    }
  }

  have_map_ = true;
  RCLCPP_INFO(
    get_logger(), "Received map: %dx%d cells @ %.3f m/cell.",
    grid_.width, grid_.height, grid_.resolution);
}

// Look up the start pose, run A*, publish the path.
void AStarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  if (!have_map_) {
    RCLCPP_WARN(get_logger(), "No map received yet; cannot plan.");
    return;
  }

  // The robot's current pose comes from TF (map -> base_footprint).
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Could not get robot pose from TF: %s", ex.what());
    return;
  }

  const double start_wx = tf.transform.translation.x;
  const double start_wy = tf.transform.translation.y;
  const double goal_wx = goal->pose.position.x;
  const double goal_wy = goal->pose.position.y;

  int start_col, start_row, goal_col, goal_row;
  if (!grid_.worldToMap(start_wx, start_wy, start_col, start_row)) {
    RCLCPP_WARN(get_logger(), "Start pose is outside the map bounds.");
    return;
  }
  if (!grid_.worldToMap(goal_wx, goal_wy, goal_col, goal_row)) {
    RCLCPP_WARN(get_logger(), "Goal pose is outside the map bounds.");
    return;
  }
  if (blocked_[grid_.index(start_col, start_row)] != 0) {
    RCLCPP_WARN(get_logger(), "Start cell is blocked (in/near an obstacle).");
    return;
  }
  if (blocked_[grid_.index(goal_col, goal_row)] != 0) {
    RCLCPP_WARN(get_logger(), "Goal cell is blocked (in/near an obstacle).");
    return;
  }

  const std::vector<int> cell_path =
    astarSearch(grid_, blocked_, start_col, start_row, goal_col, goal_row);

  if (cell_path.empty()) {
    RCLCPP_WARN(get_logger(), "No path found; publishing empty path.");
    // Publish an empty path so the follower clears any stale path.
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = now();
    plan_pub_->publish(empty);
    return;
  }

  publishPath(cell_path);
  RCLCPP_INFO(
    get_logger(), "Published path with %zu poses.", cell_path.size());
}

// Convert a cell path into a nav_msgs/Path and publish it. Each pose faces the
// next point; the final pose keeps the previous heading.
void AStarPlanner::publishPath(const std::vector<int> & cell_path) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = now();
  path.poses.reserve(cell_path.size());

  double prev_yaw = 0.0;
  for (std::size_t i = 0; i < cell_path.size(); ++i) {
    const int idx = cell_path[i];
    const int col = idx % grid_.width;
    const int row = idx / grid_.width;

    double wx, wy;
    grid_.mapToWorld(col, row, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    double yaw = prev_yaw;
    if (i + 1 < cell_path.size()) {
      const int next_idx = cell_path[i + 1];
      double nwx, nwy;
      grid_.mapToWorld(next_idx % grid_.width, next_idx / grid_.width, nwx, nwy);
      yaw = std::atan2(nwy - wy, nwx - wx);
    }
    prev_yaw = yaw;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
  }

  plan_pub_->publish(path);
}

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::AStarPlanner)
