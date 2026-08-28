#include "particle_filter/particle_filter.hpp"

#include <cmath>
#include <random>

namespace particle_filter
{

// The motion update applies the odometry delta to each particle with added noise. 
double ParticleFilter::randomUniform(double min, double max)
{
  std::uniform_real_distribution<double> dist(min, max);
  return dist(rng_);
}

// Sample from a zero-mean Gaussian distribution with the given standard deviation. This is used to add noise to the particles 
// during the motion update, with the amount of noise scaled based on the amount of movement according to odometry.
double ParticleFilter::gaussianNoise(double stddev)
{
  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng_);
}

// Wrap angle to [-pi, pi]. This is used to ensure that the particle orientations remain within a standard range, which helps with convergence and prevents issues with angle discontinuities.
double ParticleFilter::wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

} // namespace particle_filter
