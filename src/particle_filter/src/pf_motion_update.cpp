#include "particle_filter/particle_filter.hpp"

namespace particle_filter
{

void ParticleFilter::applyMotionUpdate(
  double dx_body, double dy_body, double dyaw,
  double translation, double rotation)
{
  // TODO(Lesson 9): scale the three noise standard deviations by the amount of
  // motion, then apply one noisy body-frame delta to every particle in the
  // direction of that particle's heading. Wrap the updated angle.
  // This no-op keeps the particle-filter package buildable before the lesson.
  (void)dx_body;
  (void)dy_body;
  (void)dyaw;
  (void)translation;
  (void)rotation;
}

}  // namespace particle_filter
