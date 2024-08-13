#include <mmr_base/msg/detail/ecu_status__struct.hpp>
#include <rclcpp/qos.hpp>

#include <mmr_base/configuration.hpp>

#include <control_node/estimation/inspection/inspection_estimator.hpp>

namespace control_node {
namespace estimation {
namespace inspection {

void InspectionEstimator::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters&) {
  m_ecu_status_sub = node.create_subscription<mmr_base::msg::EcuStatus>(p.get<std::string>("ecu_status.topic"), p.parse_qos("ecu_status.qos"), std::bind(&InspectionEstimator::ecu_status_cb, this, std::placeholders::_1));
  m_act_status_sub = node.create_subscription<mmr_base::msg::ActuatorStatus>(p.get<std::string>("actuator_status.topic"), p.parse_qos("actuator_status.qos"), std::bind(&InspectionEstimator::act_status_cb, this, std::placeholders::_1));
}

void InspectionEstimator::ecu_status_cb(std::shared_ptr<const mmr_base::msg::EcuStatus> msg) {
  m_state.m_gear = msg->gear;
  m_state.m_lc_is_active = msg->bool_ack_ideal_launch_control;
  m_state.m_rpm = msg->nmot;
  m_state.m_speed = (msg->wheel_speed_rear_left + msg->wheel_speed_rear_right) / 2;
}

void InspectionEstimator::act_status_cb(std::shared_ptr<const mmr_base::msg::ActuatorStatus> msg) {
  m_state.m_clutch_is_engaged = static_cast<MOTOR::ACTUATOR_STATUS>(msg->clutch_status) == MOTOR::ACTUATOR_STATUS::ENGAGE;
}

}; // namespace inspection
}; // namespace estimation
}; // namespace control_node