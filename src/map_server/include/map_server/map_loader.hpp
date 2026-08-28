// map_loader.hpp
//
// Pure file parsing for a standard ROS map (.yaml + .pgm pair): no ROS node,
// publisher, or clock involved, so it can be unit-tested on its own.

#pragma once

#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace robonav_training
{

// Loads a standard ROS map (.yaml + .pgm) into an OccupancyGrid (frame_id "map").
// Throws std::runtime_error with a descriptive message on any failure.
nav_msgs::msg::OccupancyGrid loadMapFromYaml(const std::string & yaml_path);

}  // namespace robonav_training
