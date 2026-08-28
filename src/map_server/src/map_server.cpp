// map_server.cpp
//
// MapServer node: declares the map_yaml_path parameter, owns the publisher,
// and publishes the loaded OccupancyGrid once on /map with latched QoS.

#include "map_server/map_server.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include "map_server/map_loader.hpp"

namespace robonav_training
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: Node("map_server", options)
{
  const std::string yaml_path = this->declare_parameter<std::string>("map_yaml_path", "");
  if (yaml_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'map_yaml_path' is empty; nothing to load.");
    return;
  }

  // Latched QoS: keep the last (and only) message so late subscribers get it.
  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

  // loadMapFromYaml throws std::runtime_error on failure; log it rather than crash.
  try {
    grid_ = loadMapFromYaml(yaml_path);
    grid_.header.stamp = this->now();
    map_pub_->publish(grid_);
    RCLCPP_INFO(
      this->get_logger(),
      "Published map on /map: %u x %u cells at %.3f m/cell.",
      grid_.info.width, grid_.info.height, grid_.info.resolution);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return;
  }
}

}  // namespace robonav_training

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::MapServer)
