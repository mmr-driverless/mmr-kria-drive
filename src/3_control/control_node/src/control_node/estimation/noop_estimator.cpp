#include <control_node/estimation/noop_estimator/noop_estimator.hpp>

namespace control_node {
namespace estimation {
namespace noop {

NoopEstimator::NoopEstimator(rclcpp::Node& node, const Parameters& p)
  : m_odom_sub(node.create_subscription<nav_msgs::msg::Odometry>(p.get<std::string>("odometry_topic"), rclcpp::SensorDataQoS(), std::bind(&NoopEstimator::odom_cb, this, std::placeholders::_1)))
{}

void NoopEstimator::odom_cb(nav_msgs::msg::Odometry::SharedPtr msg) {
  m_state.m_position = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  );
  auto rpy = q.toRotationMatrix().eulerAngles(0,1,2);
  m_state.m_yaw = rpy.z();
}

};
};
};