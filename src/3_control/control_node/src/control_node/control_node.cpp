#include <control_node/control_node.hpp>
#include <chrono>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>
#include <control_node/actuation/sim/sim.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/visualize.hpp>

namespace control_node {

ControlNode::ControlNode() : NodeBase("control_node"),
  m_tick_interval(std::chrono::milliseconds(this->declare_parameter("tick_interval", rclcpp::PARAMETER_INTEGER).get<int>())),
  m_vp(VehicleParameters(Parameters(this, "vehicle_parameters"))),
  m_estimator(std::make_unique<estimation::noop::NoopEstimator>(*this, Parameters(this, "noop_estimator"))),
  m_controller(std::make_unique<control::pure_pursuit_2023::PurePursuit2023>(Parameters(this, "pure_pursuit_2023"), m_vp)),
  m_centerline_sub(this->create_subscription<viz_msgs::Marker>("/planning/center_line", 1, std::bind(&ControlNode::center_line_cb, this, std::placeholders::_1))),
  m_centerline_cmpl_sub(this->create_subscription<viz_msgs::Marker>("/planning/center_line_completed", 1, std::bind(&ControlNode::center_line_completed_cb, this, std::placeholders::_1))),
  m_viz_pub(this->create_publisher<viz_msgs::MarkerArray>("/control/viz", 2))
{
  
  #ifdef USE_EDF
  this->configureEDFScheduler(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_interval()).count(),
    this->declare_parameter("wcet_ns", rclcpp::PARAMETER_INTEGER).get<int>(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_interval()).count()
  );
  #endif
  
  m_has_completed_path = false;
  m_actuators.push_back(std::make_unique<actuation::canopen_bridge::CANOpenBridge>(*this, Parameters(this, "actuation.actuators._0.params")));
  m_actuators.push_back(std::make_unique<actuation::sim::Sim>(*this, Parameters(this, "actuation.actuators._1.params")));
}


void ControlNode::center_line_cb(viz_msgs::Marker::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Centerline received");
  if (m_has_completed_path)
    return;

  m_waypoints.resize(msg->points.size());
  m_path_data.resize(msg->points.size());

  for (size_t i = 0; i < msg->points.size(); i++)
    m_waypoints[i] = Eigen::Vector2d(msg->points[i].x, msg->points[i].y);

  m_last_path_ref = {};
  m_path = path::ReferencePath(std::span<Eigen::Vector2d>(m_waypoints), std::span<path::ReferencePath::PointData::StorageT>(m_path_data), false, false, 10);
}

void ControlNode::center_line_completed_cb(viz_msgs::Marker::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Completed centerline received");

  m_has_completed_path = true;
  m_waypoints.resize(msg->points.size());
  m_path_data.resize(msg->points.size());

  for (size_t i = 0; i < msg->points.size(); i++)
    m_waypoints[i] = Eigen::Vector2d(msg->points[i].x, msg->points[i].y);

  m_last_path_ref = {};
  m_path = path::ReferencePath(std::span<Eigen::Vector2d>(m_waypoints), std::span<path::ReferencePath::PointData::StorageT>(m_path_data), true, false, 10);
}

void ControlNode::tick() {
  auto vehicle_state = m_estimator->update_and_get_current_state();

  control::Control u(0,0,0,0,0);
  auto projection = m_path.project_vehicle(vehicle_state.position, m_last_path_ref);
  std::optional<path::ReferencePath::PointRef> closest_point;
  std::optional<Eigen::Vector2d> lookforward;
  if (projection.has_value()) {
    lookforward = m_path.get_position(m_path.advance_point(projection->closest_point, m_controller->minLookForward()));
    closest_point = projection->closest_point;
    m_last_path_ref = projection->closest_point;
  }
  u = m_controller->control(vehicle_state, m_path, closest_point);

  auto msg = visualize(*this, m_path, projection, vehicle_state.position, lookforward, 10);
  m_viz_pub->publish(msg);

  for (auto& actuator : m_actuators)
    actuator->actuate(u);
}

};