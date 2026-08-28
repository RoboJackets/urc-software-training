// The A* search itself. See a_star_search.hpp and Lesson 10 for the requirements.

#include "a_star_planner/a_star_search.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace robonav_training
{
namespace
{

bool isBlocked(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int col, int row)
{
  return !grid.inBounds(col, row) || blocked[grid.index(col, row)] != 0;
}

}  // namespace

std::vector<int> astarSearch(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int start_col, int start_row, int goal_col, int goal_row)
{
  // Part 1 is provided: reject malformed inputs and impossible endpoints.
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
    blocked.size() != grid.cellCount() ||
    !grid.inBounds(start_col, start_row) || !grid.inBounds(goal_col, goal_row) ||
    isBlocked(grid, blocked, start_col, start_row) ||
    isBlocked(grid, blocked, goal_col, goal_row))
  {
    return {};
  }

  const std::size_t cell_count = grid.cellCount();
  const std::size_t start_index = grid.index(start_col, start_row);
  const std::size_t goal_index = grid.index(goal_col, goal_row);
  std::vector<double> g_score(
    cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  g_score[start_index] = 0.0;

  // TODO(Lesson 10, part 2): write the Euclidean-distance heuristic in meters.
  // TODO(Lesson 10, part 3): define an open-set entry, a lowest-f-first
  // priority queue, and push the start with f = heuristic(start).
  // TODO(Lesson 10, part 4): while open is not empty, pop the best entry,
  // discard stale entries, and mark the current cell closed.
  // TODO(Lesson 10, part 5): when the goal is popped, reconstruct the path by
  // following came_from back to the start and reverse it.
  // TODO(Lesson 10, part 6): visit eight neighbors, reject blocked cells and
  // diagonal corner cuts, then relax improved g-scores and push new entries.

  // These structures are intentionally ready for the TODO loop above. Until it
  // is implemented, returning no path is the safe, buildable behavior.
  (void)goal_index;
  (void)g_score;
  (void)came_from;
  (void)closed;
  return {};
}

}  // namespace robonav_training
