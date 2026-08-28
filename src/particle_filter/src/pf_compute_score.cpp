#include "particle_filter/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace particle_filter
{
namespace
{

constexpr int8_t kOccupiedThreshold = 50;
constexpr double kMaxObstacleLookupDistanceMeters = 1.0;
constexpr double kHugeSquaredDistanceCells = 1e12;

// Convert one measured beam into the map-frame endpoint expected for a
// candidate particle pose. This coordinate plumbing is provided so the learner
// can concentrate on the likelihood model and weight update.
struct BeamEndpoint
{
  double x;
  double y;
};

BeamEndpoint computeEndpoint(
  const Particle & particle, size_t beam_index,
  const sensor_msgs::msg::LaserScan::SharedPtr & scan,
  double base_to_lidar_x, double base_to_lidar_y,
  double base_to_lidar_yaw, bool have_extrinsic)
{
  const double beam_angle =
    scan->angle_min + static_cast<double>(beam_index) * scan->angle_increment;
  const double range = scan->ranges[beam_index];
  const double cosine = std::cos(particle.theta);
  const double sine = std::sin(particle.theta);

  const double lidar_x = have_extrinsic ?
    particle.x + cosine * base_to_lidar_x - sine * base_to_lidar_y : particle.x;
  const double lidar_y = have_extrinsic ?
    particle.y + sine * base_to_lidar_x + cosine * base_to_lidar_y : particle.y;
  const double lidar_yaw = have_extrinsic ?
    particle.theta + base_to_lidar_yaw : particle.theta;
  const double ray_yaw = lidar_yaw + beam_angle;

  return {
    lidar_x + range * std::cos(ray_yaw),
    lidar_y + range * std::sin(ray_yaw)};
}

// Squared Euclidean distance transform for one row or column. The starter keeps
// likelihood-field construction complete so Lesson 9 can focus on using it.
void distanceTransform1D(
  const std::vector<double> & input, int length, std::vector<double> & output,
  std::vector<int> & locations, std::vector<double> & boundaries)
{
  if (length <= 0) {
    return;
  }

  int k = 0;
  locations[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();

  for (int q = 1; q < length; ++q) {
    double intersection = 0.0;
    while (true) {
      const int previous = locations[k];
      const double q_cost = input[q] + static_cast<double>(q * q);
      const double previous_cost =
        input[previous] + static_cast<double>(previous * previous);
      intersection =
        (q_cost - previous_cost) / (2.0 * static_cast<double>(q - previous));
      if (intersection > boundaries[k]) {
        break;
      }
      --k;
    }
    ++k;
    locations[k] = q;
    boundaries[k] = intersection;
    boundaries[k + 1] = std::numeric_limits<double>::infinity();
  }

  k = 0;
  for (int q = 0; q < length; ++q) {
    while (boundaries[k + 1] < static_cast<double>(q)) {
      ++k;
    }
    const double delta = static_cast<double>(q - locations[k]);
    output[q] = delta * delta + input[locations[k]];
  }
}

}  // namespace

void ParticleFilter::rebuildDistanceField()
{
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double resolution = map_.info.resolution;
  const size_t cell_count = static_cast<size_t>(width) * static_cast<size_t>(height);

  if (width <= 0 || height <= 0 || resolution <= 0.0 || map_.data.size() != cell_count) {
    distance_field_m_.clear();
    have_distance_field_ = false;
    return;
  }

  std::vector<double> column_distances(cell_count, kHugeSquaredDistanceCells);
  const int max_dimension = std::max(width, height);
  std::vector<double> input(static_cast<size_t>(max_dimension));
  std::vector<double> output(static_cast<size_t>(max_dimension));
  std::vector<int> locations(static_cast<size_t>(max_dimension));
  std::vector<double> boundaries(static_cast<size_t>(max_dimension + 1));

  for (int col = 0; col < width; ++col) {
    for (int row = 0; row < height; ++row) {
      const size_t index = static_cast<size_t>(row * width + col);
      input[static_cast<size_t>(row)] =
        map_.data[index] >= kOccupiedThreshold ? 0.0 : kHugeSquaredDistanceCells;
    }
    distanceTransform1D(input, height, output, locations, boundaries);
    for (int row = 0; row < height; ++row) {
      column_distances[static_cast<size_t>(row * width + col)] =
        output[static_cast<size_t>(row)];
    }
  }

  distance_field_m_.resize(cell_count);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      input[static_cast<size_t>(col)] =
        column_distances[static_cast<size_t>(row * width + col)];
    }
    distanceTransform1D(input, width, output, locations, boundaries);
    for (int col = 0; col < width; ++col) {
      distance_field_m_[static_cast<size_t>(row * width + col)] =
        static_cast<float>(std::sqrt(output[static_cast<size_t>(col)]) * resolution);
    }
  }

  have_distance_field_ = true;
}

double ParticleFilter::distanceToNearestObstacle(double map_x, double map_y) const
{
  if (!have_distance_field_ || map_.info.resolution <= 0.0) {
    return kMaxObstacleLookupDistanceMeters;
  }

  const int col = static_cast<int>(std::floor(
      (map_x - map_.info.origin.position.x) / map_.info.resolution));
  const int row = static_cast<int>(std::floor(
      (map_y - map_.info.origin.position.y) / map_.info.resolution));
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (col < 0 || row < 0 || col >= width || row >= height) {
    return kMaxObstacleLookupDistanceMeters;
  }

  const size_t index = static_cast<size_t>(row * width + col);
  if (index >= distance_field_m_.size() || !std::isfinite(distance_field_m_[index])) {
    return kMaxObstacleLookupDistanceMeters;
  }
  return std::min(
    static_cast<double>(distance_field_m_[index]),
    kMaxObstacleLookupDistanceMeters);
}

void ParticleFilter::score(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg || !have_map_ || !have_distance_field_ || particles_.empty()) {
    return;
  }

  double base_to_lidar_x = 0.0;
  double base_to_lidar_y = 0.0;
  double base_to_lidar_yaw = 0.0;
  const bool have_extrinsic = lookupBaseToLidar(
    base_to_lidar_x, base_to_lidar_y, base_to_lidar_yaw);

  std::vector<double> log_scores(particles_.size(), 0.0);
  size_t valid_beam_count = 0;

  const double range_max = std::max(static_cast<double>(msg->range_max), 1e-6);
  const double sigma = std::max(sigma_hit_, 1e-6);
  constexpr double kPi = 3.14159265358979323846;
  const double log_normalization = -std::log(std::sqrt(2.0 * kPi) * sigma);
  const double inverse_two_sigma_squared = 1.0 / (2.0 * sigma * sigma);
  const double log_z_hit = std::log(std::max(z_hit_, 1e-12));
  const double log_z_random = std::log(std::max(z_rand_, 1e-12));
  const double log_uniform_random = std::log(1.0 / range_max);

  for (size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
    double accumulated_log_score = 0.0;

    for (size_t beam_index = 0; beam_index < msg->ranges.size();
      beam_index += static_cast<size_t>(beam_stride_))
    {
      const double range = msg->ranges[beam_index];
      if (!std::isfinite(range) || range < msg->range_min ||
        range >= msg->range_max * 0.99)
      {
        continue;
      }

      if (particle_index == 0) {
        ++valid_beam_count;
      }

      const auto endpoint = computeEndpoint(
        particles_[particle_index], beam_index, msg,
        base_to_lidar_x, base_to_lidar_y, base_to_lidar_yaw, have_extrinsic);
      const double obstacle_distance =
        distanceToNearestObstacle(endpoint.x, endpoint.y);

      const double log_hit =
        log_z_hit + log_normalization -
        obstacle_distance * obstacle_distance * inverse_two_sigma_squared;
      const double log_random = log_z_random + log_uniform_random;
      const double largest = std::max(log_hit, log_random);
      accumulated_log_score += largest + std::log(
        std::exp(log_hit - largest) + std::exp(log_random - largest));
    }

    log_scores[particle_index] = accumulated_log_score;
  }

  if (valid_beam_count == 0) {
    return;
  }

  const double maximum_log_score =
    *std::max_element(log_scores.begin(), log_scores.end());
  double total_weight = 0.0;
  for (size_t index = 0; index < particles_.size(); ++index) {
    particles_[index].weight *=
      std::exp(log_scores[index] - maximum_log_score);
    total_weight += particles_[index].weight;
  }

  if (std::isfinite(total_weight) && total_weight > 1e-12) {
    for (auto & particle : particles_) {
      particle.weight /= total_weight;
    }
  } else {
    const double uniform_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.weight = uniform_weight;
    }
  }
}

}  // namespace particle_filter
