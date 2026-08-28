// map_server.hpp
//
// MapServer node: loads a static occupancy grid from a ROS map (.yaml + .pgm)
// and publishes it once as a latched nav_msgs/OccupancyGrid on /map.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robonav_training
{

class MapServer : public rclcpp::Node
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  nav_msgs::msg::OccupancyGrid grid_;  // stored so transient-local QoS can re-deliver it
};

}  // namespace robonav_training
