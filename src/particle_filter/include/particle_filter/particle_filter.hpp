#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <random>
#include <string>
#include <utility>
#include <vector>

namespace particle_filter
{

struct Particle
{
  double x;
  double y;
  double theta;
  double weight;
};

class ParticleFilter : public rclcpp::Node
{
public:
  explicit ParticleFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  double randomUniform(double min, double max);
  double gaussianNoise(double stddev);
  double wrapAngle(double a);
  void applyMotionUpdate(
    double dx_body, double dy_body, double dyaw,
    double translation, double rotation);
  bool lookupOdomToBase(double & x, double & y, double & yaw);
  bool lookupBaseToLidar(double & x, double & y, double & yaw);
  void score(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void rebuildDistanceField();
  double distanceToNearestObstacle(double map_x, double map_y) const;
  void broadCastMapToOdomTf(const rclcpp::Time & stamp);
  void systematicResample();
  void roughen();
  double effectiveSampleSize() const;
  void globalLocalization();

  std::vector<Particle> particles_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid map_;
  std::vector<float> distance_field_m_;
  rclcpp::Time last_odom_time_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mt19937 rng_;
  bool have_map_ = false;
  bool have_distance_field_ = false;

  int num_particles_;
  double particles_x_initial_;
  double particles_y_initial_;
  double particles_theta_initial_;
  double x_noise_;
  double y_noise_;
  double theta_noise_;
  double random_particle_percent_;
  bool initialized_particles_ = false;
  bool have_odom_baseline_ = false;
  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  double last_odom_yaw_ = 0.0;
  int beam_stride_ = 3;
  double init_x_;
  double init_y_;
  double init_yaw_;
  double sigma_hit_;
  double z_hit_;
  double z_rand_;
  std::string base_frame_{"base_link"};
  std::string lidar_frame_{"base_laser"};
  std::vector<std::pair<double, double>> free_cells_;
};

}  // namespace particle_filter

#endif  // PARTICLE_FILTER_HPP
