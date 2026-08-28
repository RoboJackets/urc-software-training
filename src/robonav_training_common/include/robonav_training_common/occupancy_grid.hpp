// occupancy_grid.hpp
// ROS-free occupancy grid geometry and coordinate conversions
// (world <-> cell, cell -> flat index, bounds checking).
//
// Conventions: `col` indexes the X axis, `row` indexes the Y axis;
// the flat index is row * width + col (row-major, matching
// nav_msgs/OccupancyGrid::data).

#pragma once

#include <cmath>
#include <cstddef>

namespace robonav_training
{

struct GridMap
{
  int width = 0;            // cells along X
  int height = 0;           // cells along Y
  double resolution = 0.0;  // meters per cell
  double origin_x = 0.0;    // map-frame X of lower-left corner [m]
  double origin_y = 0.0;    // map-frame Y of lower-left corner [m]

  // Map-frame point (meters) -> cell (col, row); false if outside the grid.
  bool worldToMap(double wx, double wy, int & col, int & row) const
  {
    col = static_cast<int>(std::floor((wx - origin_x) / resolution));
    row = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return inBounds(col, row);
  }

  // Cell (col, row) -> map-frame coordinates of that cell's center (meters).
  void mapToWorld(int col, int row, double & wx, double & wy) const
  {
    wx = origin_x + (static_cast<double>(col) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(row) + 0.5) * resolution;
  }

  bool inBounds(int col, int row) const
  {
    return col >= 0 && row >= 0 && col < width && row < height;
  }

  std::size_t index(int col, int row) const
  {
    return static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
           static_cast<std::size_t>(col);
  }

  std::size_t cellCount() const
  {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  }
};

}  // namespace robonav_training
