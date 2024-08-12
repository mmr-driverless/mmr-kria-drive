#include <rclcpp/qos.hpp>

#include <mmr_base/configuration.hpp>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>

namespace control_node {
namespace estimation {
namespace noop {

void NoopEstimator::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters&) {
  m_odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(p.get<std::string>("odometry_topic"), rclcpp::SensorDataQoS(), std::bind(&NoopEstimator::odom_cb, this, std::placeholders::_1));
}

void NoopEstimator::ecu_status_cb(std::shared_ptr<const mmr_base::msg::EcuStatus> msg) {
  m_state.m_gear = msg->gear;
  m_state.m_lc_is_active = msg->bool_ack_ideal_launch_control;
  m_state.m_rpm = msg->nmot;
  m_state.m_speed = msg->vehicle_speed;
}

void NoopEstimator::act_status_cb(std::shared_ptr<const mmr_base::msg::ActuatorStatus> msg) {
  m_state.m_clutch_is_engaged = static_cast<MOTOR::ACTUATOR_STATUS>(msg->clutch_status) == MOTOR::ACTUATOR_STATUS::ENGAGE;
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

};
};
};