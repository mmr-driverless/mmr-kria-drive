#include "control_node/estimation/ivehicle_state.hpp"
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
  m_refpath_mgr(*this, Parameters(this, "reference_path_manager"), this->get_logger().get_child("RefPathMgr")),
  m_estimator(std::make_unique<estimation::noop::NoopEstimator>(*this, Parameters(this, "noop_estimator"))),
  m_controller(std::make_unique<control::pure_pursuit_2023::PurePursuit2023>(Parameters(this, "pure_pursuit_2023"), m_vp)),
  m_startStop(std::make_unique<start_stop::StartStop>(Parameters(this, "start_stop"))),
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

void ControlNode::tick() {
  const estimation::IVehicleState& vehicle_state = m_estimator->update_and_get_current_state();

  control::Control u(0,0,0,0,0, false);
  path::ReferencePath path = m_refpath_mgr.get();
  if (m_refpath_mgr.changed())
    m_last_path_ref = std::nullopt;

  auto projection = path.project_vehicle(vehicle_state.position(), m_last_path_ref);
  std::optional<path::ReferencePath::PointRef> closest_point;
  if (projection.has_value()) {
    closest_point = projection->closest_point;
    m_last_path_ref = projection->closest_point;
  }
  u = m_controller->control(vehicle_state, path, closest_point);
  m_startStop->triggerFSM(vehicle_state, u);

  for (auto& actuator : m_actuators)
    actuator->actuate(u);
}

};