// Pure A* grid search (no ROS): operates on a GridMap plus a flat `blocked`
// mask and returns flat cell indices from start to goal.

#pragma once

#include "robonav_training_common/occupancy_grid.hpp"

#include <cstdint>
#include <vector>

namespace robonav_training
{

// 8-connected A* over `blocked` (nonzero == impassable). Diagonal moves are
// forbidden when they would cut a blocked corner. Returns flat cell indices
// start->goal, or an empty vector if the goal is unreachable.
std::vector<int> astarSearch(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int start_col, int start_row, int goal_col, int goal_row);

}  // namespace robonav_training
