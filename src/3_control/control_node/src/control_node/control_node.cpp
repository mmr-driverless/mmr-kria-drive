#include <control_node/control_node.hpp>
#include <chrono>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>

namespace control_node {

ControlNode::ControlNode() : rclcpp::Node("control_node"),
  m_tick_interval(std::chrono::milliseconds(this->declare_parameter("tick_interval", rclcpp::PARAMETER_INTEGER).get<int>())),
  m_vp(VehicleParameters(Parameters(this, "vehicle_parameters"))),
  m_estimator(std::make_unique<estimation::noop::NoopEstimator>(*this, Parameters(this, "noop_estimator"))),
  m_controller(std::make_unique<control::pure_pursuit_2023::PurePursuit2023>(Parameters(this, "pure_pursuit_2023"), m_vp))
{
  m_actuators.push_back(std::make_unique<actuation::canopen_bridge::CANOpenBridge>(*this, Parameters(this, "actuation.actuators._0.params")));
}

void ControlNode::tick() {
  RCLCPP_INFO(this->get_logger(), "Tick");

  auto vehicle_state = m_estimator->update_and_get_current_state();

  control::Control u(0,0,0,0,0);
  // m_controller->control(vehicle_state, );

  for (auto& actuator : m_actuators)
    actuator->actuate(u);
}

};