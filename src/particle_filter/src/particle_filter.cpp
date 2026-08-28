#include "particle_filter/particle_filter.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"

namespace particle_filter
{

ParticleFilter::ParticleFilter(const rclcpp::NodeOptions &options)
    : rclcpp::Node("particle_filter", options),
      last_odom_time_(0, 0, RCL_ROS_TIME),
      rng_(std::random_device{}())
{
  // total number of particles at any given time; tuned based on the tradeoff between localization accuracy and computational load.
  num_particles_ = std::max<int>(1, declare_parameter<int>("num_particles", 250)); 

  // Initial random spread around a local pose estimate. In the normal full-stack
  // launch, the map callback replaces this temporary prior with global localization.
  particles_x_initial_ = declare_parameter<double>("particles_x_initial", 0.25);
  particles_y_initial_ = declare_parameter<double>("particles_y_initial", 0.25);
  particles_theta_initial_ = declare_parameter<double>("particles_theta_initial", 0.15);
  init_x_ = declare_parameter<double>("init_x", 0.0);
  init_y_ = declare_parameter<double>("init_y", 0.0);
  init_yaw_ = declare_parameter<double>("init_yaw", 0.0);

  // noise added to particles during motion update; tuned based on expected odometry error and to prevent particle deprivation while still allowing convergence.
  x_noise_ = declare_parameter<double>("x_noise", 0.05);
  y_noise_ = declare_parameter<double>("y_noise", 0.075);
  theta_noise_ = declare_parameter<double>("theta_noise", 0.05);
  random_particle_percent_ =
      std::clamp(declare_parameter<double>("random_particle_percent", 5.0), 0.0, 100.0);

  // how many laser scan beams to skip when scoring particles; tuned based on the tradeoff between localization accuracy and computational load.
  beam_stride_ = std::max<int>(1, declare_parameter<int>("beam_stride", 3));

  // likelihood field model parameters; tuned based on the expected sensor noise characteristics and to balance the influence of good vs bad measurements.
  z_hit_ = declare_parameter<double>("z_hit", 0.7);
  z_rand_ = declare_parameter<double>("z_rand", 0.3);
  sigma_hit_ = std::max(1e-6, declare_parameter<double>("sigma_hit", 0.10));

  // Frames
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "base_laser");

  // IMPORTANT: particles represent BASE pose in MAP
  // (p.x, p.y, p.theta) == map->base_link

  // Create a temporary local prior. Once /map arrives, globalLocalization()
  // scatters these particles over all free cells.
  particles_.resize(static_cast<size_t>(num_particles_));
  for (auto &p : particles_) {
    p.x = init_x_ + randomUniform(-particles_x_initial_, particles_x_initial_);
    p.y = init_y_ + randomUniform(-particles_y_initial_, particles_y_initial_);
    p.theta = wrapAngle(
      init_yaw_ + randomUniform(-particles_theta_initial_, particles_theta_initial_));
    p.weight = 1.0 / static_cast<double>(num_particles_);
  }

  const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/odom/filtered");
  const std::string particle_cloud_topic = declare_parameter<std::string>("particle_cloud_topic", "/particle_cloud");
  const std::string estimated_pose_topic = declare_parameter<std::string>("estimated_pose_topic", "/estimated_pose");
  const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
  const std::string map_topic = declare_parameter<std::string>("map_topic", "/map");

  //subscribers and publishers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&ParticleFilter::odomCallback, this, std::placeholders::_1));

  particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      particle_cloud_topic, rclcpp::SystemDefaultsQoS());

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      estimated_pose_topic, rclcpp::SystemDefaultsQoS());

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&ParticleFilter::scanCallback, this, std::placeholders::_1));

  rclcpp::QoS map_qos(1);
  map_qos.transient_local().reliable();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = *msg;
      rebuildDistanceField();
      have_map_ = true;
      initialized_particles_ = false; // force re-init on new map
      have_odom_baseline_ = false;     // first odometry sample sets the new baseline
      globalLocalization();           // scatter across whole map
    });
  
  // TF setup
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void ParticleFilter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto current_time = rclcpp::Time(msg->header.stamp);

  // Use /odom/filtered as base source; override with TF odom->base if available.
  double odom_x = msg->pose.pose.position.x;
  double odom_y = msg->pose.pose.position.y;
  tf2::Quaternion odom_q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);
  double odom_yaw = tf2::getYaw(odom_q);
  (void)lookupOdomToBase(odom_x, odom_y, odom_yaw);

   
  if (!initialized_particles_) {
    globalLocalization();
  }

  if (!have_odom_baseline_) {
    last_odom_x_ = odom_x;
    last_odom_y_ = odom_y;
    last_odom_yaw_ = odom_yaw;
    last_odom_time_ = current_time;
    have_odom_baseline_ = true;
  } else {
    const double dt = (current_time - last_odom_time_).seconds();
    if (!(dt > 0.0)) return;


    // Compute odometry delta in the robot's local frame. The odometry message gives us the change in pose in the global frame, 
    // but we need to apply this change to each particle based on its own heading.
    const double dx = odom_x - last_odom_x_;
    const double dy = odom_y - last_odom_y_;
    const double dyaw = wrapAngle(odom_yaw - last_odom_yaw_);

    // Imagine the robot is at position 0,0 and facing along the positive x-axis. 
    // If the odometry says it moved dx=1 forward and dy=0 to the left, then in the robot's local frame, 
    // it moved 1 forward and 0 left. But if the robot is actually facing 90 degrees to the left 
    // (positive y-axis), then that same odometry (dx=1 forward, dy=0 left) would mean it moved 0 forward 
    // and -1 to the left in its local frame. The following rotation transforms the global odometry 
    // delta into the robot's local frame, which is what we want to apply to the particles.
    const double c = std::cos(last_odom_yaw_);
    const double s = std::sin(last_odom_yaw_);
    const double dx_body = c * dx + s * dy;
    const double dy_body = -s * dx + c * dy;


    // Total translation and rotation according to odometry; used for noise scaling.
    const double trans = std::hypot(dx, dy);
    const double rot = std::abs(dyaw);

    // Deadband: if odom barely moved, do not diffuse particles.
    const bool moved = !(trans < 1e-4 && rot < 1e-4);
    if (moved) {
      applyMotionUpdate(dx_body, dy_body, dyaw, trans, rot);

      if (!free_cells_.empty() && random_particle_percent_ > 0.0) {
        const size_t inject_count = static_cast<size_t>(
            std::round(random_particle_percent_ / 100.0 * static_cast<double>(particles_.size())));
        const double res = map_.info.resolution;
        for (size_t i = 0; i < inject_count; ++i) {
          size_t idx = static_cast<size_t>(
              randomUniform(0.0, static_cast<double>(particles_.size())));
          if (idx >= particles_.size()) {
            idx = particles_.size() - 1;
          }

          const auto &cell = free_cells_[static_cast<size_t>(
              randomUniform(0.0, static_cast<double>(free_cells_.size())))];
          particles_[idx].x = cell.first + randomUniform(-res * 0.5, res * 0.5);
          particles_[idx].y = cell.second + randomUniform(-res * 0.5, res * 0.5);
          particles_[idx].theta = wrapAngle(randomUniform(-M_PI, M_PI));
          particles_[idx].weight = 1.0 / static_cast<double>(particles_.size());
        }
      }
    }

    last_odom_x_ = odom_x;
    last_odom_y_ = odom_y;
    last_odom_yaw_ = odom_yaw;
    last_odom_time_ = current_time;
  }

  // Publish particle cloud (poses are in MAP)
  geometry_msgs::msg::PoseArray particles_msg;
  particles_msg.header = msg->header;
  particles_msg.header.frame_id = "map";
  particles_msg.poses.resize(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i) {
    const auto &p = particles_[i];
    particles_msg.poses[i].position.x = p.x;
    particles_msg.poses[i].position.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.theta);
    particles_msg.poses[i].orientation.x = q.x();
    particles_msg.poses[i].orientation.y = q.y();
    particles_msg.poses[i].orientation.z = q.z();
    particles_msg.poses[i].orientation.w = q.w();
  }
  particles_pub_->publish(particles_msg);
}


void ParticleFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 1) measurement update
  score(msg);

  // 2) publish map->odom using current particle estimate
  broadCastMapToOdomTf(rclcpp::Time(msg->header.stamp));

  // 3) publish estimated pose (map->base) as weighted mean
  geometry_msgs::msg::PoseWithCovarianceStamped est_pose;
  est_pose.header = msg->header;
  est_pose.header.frame_id = "map";

  double est_x = 0.0, est_y = 0.0;
  double sum_sin = 0.0, sum_cos = 0.0;
  for (const auto &p : particles_) {
    est_x += p.x * p.weight;
    est_y += p.y * p.weight;
    sum_sin += std::sin(p.theta) * p.weight;
    sum_cos += std::cos(p.theta) * p.weight;
  }
  const double est_yaw = std::atan2(sum_sin, sum_cos);

  double var_x = 0.0;
  double var_y = 0.0;
  double var_yaw = 0.0;
  for (const auto &p : particles_) {
    var_x += p.weight * (p.x - est_x) * (p.x - est_x);
    var_y += p.weight * (p.y - est_y) * (p.y - est_y);
    const double dyaw = wrapAngle(p.theta - est_yaw);
    var_yaw += p.weight * dyaw * dyaw;
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, est_yaw);

  est_pose.pose.pose.position.x = est_x;
  est_pose.pose.pose.position.y = est_y;
  est_pose.pose.pose.position.z = 0.0;
  est_pose.pose.pose.orientation.x = q.x();
  est_pose.pose.pose.orientation.y = q.y();
  est_pose.pose.pose.orientation.z = q.z();
  est_pose.pose.pose.orientation.w = q.w();
  est_pose.pose.covariance[0] = var_x;
  est_pose.pose.covariance[7] = var_y;
  est_pose.pose.covariance[35] = var_yaw;

  pose_pub_->publish(est_pose);

  // 4) resample only when needed (for next cycle)
  const double neff = effectiveSampleSize();
  if (neff < 0.5 * particles_.size()) {
    systematicResample();
    roughen();
  }
}

void ParticleFilter::globalLocalization()
{
  const double ox = map_.info.origin.position.x;
  const double oy = map_.info.origin.position.y;
  const double res = map_.info.resolution;
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);

  free_cells_.clear();

  // Collect all free cells
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const size_t idx = static_cast<size_t>(y) * width + x;
      if (map_.data[idx] >= 0 && map_.data[idx] < 50) { // free cell
        free_cells_.emplace_back(
          ox + (x + 0.5) * res,
          oy + (y + 0.5) * res
        );
      }
    }
  }

  if (free_cells_.empty()) return;

  for (auto &p : particles_) {
    const auto &cell = free_cells_[static_cast<size_t>(
      randomUniform(0, static_cast<double>(free_cells_.size()))
    )];
    p.x = cell.first + randomUniform(-res, res);
    p.y = cell.second + randomUniform(-res, res);
    p.theta = randomUniform(-M_PI, M_PI);
    p.weight = 1.0 / static_cast<double>(num_particles_);
  }
  initialized_particles_ = true;
}

} // namespace particle_filter

RCLCPP_COMPONENTS_REGISTER_NODE(particle_filter::ParticleFilter)
