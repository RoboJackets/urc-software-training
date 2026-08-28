// pure_pursuit_core.hpp
// ROS-free Pure Pursuit path-tracking geometry.
//
// Pure pursuit drives the robot along the circular arc that connects it to a
// "lookahead" point a fixed distance ahead on the path. With the lookahead
// point in the robot frame (x_r forward, y_r left), that arc has curvature
//     curvature = 2 * y_r / Ld^2     where Ld = hypot(x_r, y_r)
// and the diff-drive turn rate is linear velocity * curvature.

#pragma once

#include <array>
#include <vector>

namespace robonav_training
{

// The velocity command this algorithm produces.
struct PursuitCommand
{
  double linear;       // forward speed (m/s)
  double angular;      // turn rate (rad/s), positive = left
  bool goal_reached;   // true once within goal_tolerance of the final point
};

// Compute the velocity command for the given robot pose and polyline path
// (each element {x, y} in the map frame). Returns goal_reached = true with a
// stop command when within goal_tolerance of the final point. An empty path
// returns a stop command with goal_reached = false.
PursuitCommand computePursuitCommand(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<std::array<double, 2>> & path,
  double lookahead_distance, double desired_linear_speed,
  double max_angular_speed, double goal_tolerance);

}  // namespace robonav_training
