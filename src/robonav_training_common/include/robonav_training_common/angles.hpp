// angles.hpp
// Shared, header-only helpers for keeping angles bounded.

#pragma once

#include <cmath>

namespace robonav_training
{

// Wrap any angle into (-pi, pi]. Needed before storing a heading or forming an
// innovation, so a +179 vs -179 deg wrap reads as 2 deg, not 358 deg.
// atan2(sin, cos) does this branch-free via the periodicity of sin/cos.
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace robonav_training
