#include <rclcpp/qos.hpp>

#include <mmr_base/configuration.hpp>

#include <control_node/estimation/sim_estimator/sim_estimator.hpp>

namespace control_node {
namespace estimation {
namespace sim {

void SimEstimator::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters&) {
  m_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(p.get<std::string>("odometry_topic"), rclcpp::SensorDataQoS(), std::bind(&SimEstimator::odom_cb, this, std::placeholders::_1));
}

static inline double yaw_from_quaternion(const Eigen::Quaterniond& q)
{
  return std::atan2(2.0 * (q.z() * q.w() + q.x() * q.y()) , -1.0 + 2.0 * (q.w() * q.w() + q.x() * q.x()));
}

void SimEstimator::odom_cb(std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  m_state.m_position = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  ); 

  m_state.m_speed = msg->twist.twist.linear.x;
  m_state.m_yaw_rate = msg->twist.twist.angular.z;
  auto rpy = yaw_from_quaternion(q);
  m_state.m_yaw = rpy;
}

}; // namespace noop
}; // namespace estimation
}; // namespace control_node
