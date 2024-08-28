#include <rclcpp/qos.hpp>

#include <mmr_base/configuration.hpp>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>

namespace control_node {
namespace estimation {
namespace noop {

void NoopEstimator::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters&) {
  m_clutch_disengaged_thresh = p.get<double>("clutch_disengaged_thresh");

  m_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(p.get<std::string>("odometry_topic"), rclcpp::SensorDataQoS(), std::bind(&NoopEstimator::odom_cb, this, std::placeholders::_1));
  m_ecu_status_sub = node.create_subscription<mmr_base::msg::EcuStatus>(p.get<std::string>("ecu_status.topic"), p.parse_qos("ecu_status.qos"), std::bind(&NoopEstimator::ecu_status_cb, this, std::placeholders::_1));
  m_res_status_sub = node.create_subscription<mmr_base::msg::ResStatus>(p.get<std::string>("res_status.topic"), p.parse_qos("res_status.qos"), std::bind(&NoopEstimator::res_status_cb, this, std::placeholders::_1));
}

void NoopEstimator::ecu_status_cb(std::shared_ptr<const mmr_base::msg::EcuStatus> msg) {
  m_state.m_gear = msg->gear;
  m_state.m_lc_is_active = msg->bool_ack_ideal_launch_control;
  m_state.m_rpm = msg->nmot;
  m_state.m_speed = msg->vehicle_speed / 3.6;
  m_state.m_clutch_is_engaged = msg->clutch_percentage < m_clutch_disengaged_thresh;
  m_state.m_steering_angle = msg->steering_angle;
  m_state.m_throttle = msg->throttle;
}

void NoopEstimator::odom_cb(std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
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

void NoopEstimator::res_status_cb(std::shared_ptr<const mmr_base::msg::ResStatus> msg) {
  m_state.m_res_go = msg->go_signal;
  m_state.m_res_bag = msg->bag;
}

}; // namespace noop
}; // namespace estimation
}; // namespace control_node