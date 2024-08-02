#include <control_node/control_node.hpp>
#include <chrono>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>
#include <control_node/path/reference_path.hpp>

namespace control_node {

ControlNode::ControlNode() : rclcpp::Node("control_node"),
  m_tick_interval(std::chrono::milliseconds(this->declare_parameter("tick_interval", rclcpp::PARAMETER_INTEGER).get<int>())),
  m_vp(VehicleParameters(Parameters(this, "vehicle_parameters"))),
  m_estimator(std::make_unique<estimation::noop::NoopEstimator>(*this, Parameters(this, "noop_estimator"))),
  m_controller(std::make_unique<control::pure_pursuit_2023::PurePursuit2023>(Parameters(this, "pure_pursuit_2023"), m_vp))
{
  m_actuators.push_back(std::make_unique<actuation::canopen_bridge::CANOpenBridge>(*this, Parameters(this, "actuation.actuators._0.params")));
}


void ControlNode::center_line_cb(mmr_kria_base::msg::Marker::SharedPtr msg) {
  m_waypoints.resize(msg->points.size());
  m_path_data.resize(msg->points.size());

  for (size_t i = 0; i < msg->points.size(); i++) {
    m_waypoints[i] = Eigen::Vector2d(msg->points[i].x, msg->points[i].y);
    // m_path_data[i].curvature = msg->points[i].z;
  }

  m_path = path::ReferencePath(std::span<Eigen::Vector2d>(m_waypoints), std::span<path::ReferencePath::PointData::StorageT>(m_path_data), false, false, 10);
}

void ControlNode::center_line_completed_cb(mmr_kria_base::msg::Marker::SharedPtr msg) {
  m_waypoints.resize(msg->points.size());
  m_path_data.resize(msg->points.size());

  for (size_t i = 0; i < msg->points.size(); i++) {
    m_waypoints[i] = Eigen::Vector2d(msg->points[i].x, msg->points[i].y);
    // m_path_data[i].curvature = msg->points[i].z;
  }

  m_path = path::ReferencePath(std::span<Eigen::Vector2d>(m_waypoints), std::span<path::ReferencePath::PointData::StorageT>(m_path_data), true, false, 10);
}

void ControlNode::tick() {
  RCLCPP_INFO(this->get_logger(), "Tick");

  // path::ReferencePath m_path;
  
  auto vehicle_state = m_estimator->update_and_get_current_state();

  control::Control u(0,0,0,0,0);
  auto projection = m_path.project_vehicle(vehicle_state.position, m_last_path_ref);
  if (!projection.has_value()) {};

  // m_last_path_ref = *projection;
  

  // m_controller->control(vehicle_state, );

  for (auto& actuator : m_actuators)
    actuator->actuate(u);
}

};