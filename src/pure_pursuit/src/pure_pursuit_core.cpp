// Pure Pursuit geometry. See pure_pursuit_core.hpp and Lesson 11.

#include "pure_pursuit/pure_pursuit_core.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

namespace robonav_training
{

PursuitCommand computePursuitCommand(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<std::array<double, 2>> & path,
  double lookahead_distance, double desired_linear_speed,
  double max_angular_speed, double goal_tolerance)
{
  // Part 1 is provided: no path means there is no active goal to follow.
  if (path.empty()) {
    return PursuitCommand{0.0, 0.0, false};
  }

  // Part 2 is provided: stop and report completion inside the goal tolerance.
  const auto & goal = path.back();
  const double distance_to_goal =
    std::hypot(goal[0] - robot_x, goal[1] - robot_y);
  if (distance_to_goal <= goal_tolerance) {
    return PursuitCommand{0.0, 0.0, true};
  }

  // Part 3 is provided: find where the robot is closest to the path. The
  // lookahead search should begin here so it does not pull toward old points.
  std::size_t nearest_index = 0;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double dx = path[index][0] - robot_x;
    const double dy = path[index][1] - robot_y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = index;
    }
  }

  // TODO(Lesson 11, part 4): search forward from nearest_index for the first
  // point at least lookahead_distance away; fall back to path.back().
  // TODO(Lesson 11, part 5): translate and rotate that point into robot-frame
  // coordinates x_r (forward) and y_r (left).
  // TODO(Lesson 11, part 6): handle a point behind the robot; otherwise compute
  // curvature, clamp angular speed, and slow down near the goal.

  (void)robot_yaw;
  (void)lookahead_distance;
  (void)desired_linear_speed;
  (void)max_angular_speed;
  (void)nearest_index;
  return PursuitCommand{0.0, 0.0, false};
}

}  // namespace robonav_training
