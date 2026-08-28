#include "particle_filter/particle_filter.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <cmath>

namespace particle_filter
{

bool ParticleFilter::lookupOdomToBase(double &x, double &y, double &yaw)
{
  if (!tf_buffer_) return false;

  try {
    const auto tf = tf_buffer_->lookupTransform("odom", base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    yaw = tf2::getYaw(q);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// Returns BASE -> LIDAR as (dx, dy, dyaw) expressed in BASE frame.
bool ParticleFilter::lookupBaseToLidar(double &x, double &y, double &yaw)
{
  if (!tf_buffer_) return false;

  try {
    // target=base, source=lidar => base->lidar (lidar origin in base frame)
    const auto tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    yaw = tf2::getYaw(q);
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed TF lookup base->lidar (%s->%s): %s",
                         base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    return false;
  }
}

void ParticleFilter::broadCastMapToOdomTf(const rclcpp::Time &stamp)
{
  if (!tf_broadcaster_) return;
  if (!initialized_particles_) return;

  const double odom_x = last_odom_x_;
  const double odom_y = last_odom_y_;
  const double odom_yaw = last_odom_yaw_;

  // map->base estimated from particles (weighted mean)
  double est_x = 0.0, est_y = 0.0;
  double sum_sin = 0.0, sum_cos = 0.0;
  for (const auto &p : particles_) {
    est_x += p.x * p.weight;
    est_y += p.y * p.weight;
    sum_sin += std::sin(p.theta) * p.weight;
    sum_cos += std::cos(p.theta) * p.weight;
  }
  const double est_yaw = std::atan2(sum_sin, sum_cos);

  tf2::Transform map_to_base;
  map_to_base.setOrigin(tf2::Vector3(est_x, est_y, 0.0));
  tf2::Quaternion q_map;
  q_map.setRPY(0.0, 0.0, est_yaw);
  map_to_base.setRotation(q_map);

  tf2::Transform odom_to_base;
  odom_to_base.setOrigin(tf2::Vector3(odom_x, odom_y, 0.0));
  tf2::Quaternion q_odom;
  q_odom.setRPY(0.0, 0.0, odom_yaw);
  odom_to_base.setRotation(q_odom);

  // map->odom = map->base * (odom->base)^-1
  const tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  tf_msg.transform.translation.x = map_to_odom.getOrigin().x();
  tf_msg.transform.translation.y = map_to_odom.getOrigin().y();
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = map_to_odom.getRotation().x();
  tf_msg.transform.rotation.y = map_to_odom.getRotation().y();
  tf_msg.transform.rotation.z = map_to_odom.getRotation().z();
  tf_msg.transform.rotation.w = map_to_odom.getRotation().w();

  tf_broadcaster_->sendTransform(tf_msg);
}

} // namespace particle_filter
