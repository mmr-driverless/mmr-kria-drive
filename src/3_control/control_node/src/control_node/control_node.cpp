#include <chrono>
#include <stdexcept>

#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/parameters.hpp>
#include <control_node/control_node.hpp>

#include <control_node/path/reference_path.hpp>
#include <control_node/visualize.hpp>

#include <control_node/actuation/actuator_factory.hpp>
#include <control_node/estimation/state_estimator_factory.hpp>
#include "control_node/control/controller_factory.hpp"


namespace control_node {

ControlNode::ControlNode() : NodeBase("control_node"),
  m_tick_interval(std::chrono::milliseconds(this->declare_parameter("tick_interval", rclcpp::PARAMETER_INTEGER).get<int>())),
  m_vp(VehicleParameters(Parameters(this, "vehicle_parameters"))),
  m_refpath_mgr(*this, Parameters(this, "reference_path_manager"), this->get_logger().get_child("RefPathMgr")),
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

  setup_actuators();
  setup_estimator();
  setup_controller();
}

void ControlNode::setup_estimator() {
  Parameters p(this, "estimation");
  auto type = p.get<std::string>("type");
  m_estimator = estimation::get_factory().get(type);
  if (m_estimator == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "UNKNOWN estimator type '%s'", type.c_str());
    throw std::runtime_error("Bad estimator type");
  }

  RCLCPP_INFO(this->get_logger(), "INITIALIZING estimator '%s'.", type.c_str());
  m_estimator->init(*this, p.subparams("params"), m_vp);
}
void ControlNode::setup_controller() {
  Parameters p(this, "control");
  auto type = p.get<std::string>("type");
  m_controller = control::get_factory().get(type);
  if (m_controller == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "UNKNOWN controller type '%s'.", type.c_str());
    throw std::runtime_error("Bad controller type");
  }
  
  RCLCPP_INFO(this->get_logger(), "INITIALIZING controller '%s'.", type.c_str());
  m_controller->init(*this, p.subparams("params"), m_vp);
}
void ControlNode::setup_actuators() {
  auto logger = this->get_logger().get_child("setup_actuators");

  Parameters p(this, "actuation");
  m_actuators = actuation::get_factory().from_param_list(p, "actuators", [this](actuation::IActuator& act, int, const Parameters& p_i) {
    act.init(*this, p_i);
  }, logger);

  if (m_actuators.size() == 0)
    RCLCPP_WARN(logger, "NO actuators initialized!");
  else
    RCLCPP_INFO(logger, "INITIALIZED %zu actuators.", m_actuators.size());
}

void ControlNode::tick() {
  const estimation::IVehicleState& vehicle_state = m_estimator->update_and_get_current_state();

  control::Control u(0,0,0,0,0, false);
  path::ReferencePath path = m_refpath_mgr.get();
  if (m_refpath_mgr.changed())
    m_last_path_ref = std::nullopt;

  auto projection = path.project_vehicle(vehicle_state.position(), m_last_path_ref, 8.0 * 8.0);
  std::optional<path::ReferencePath::PointRef> closest_point;
  if (projection.has_value()) {
    closest_point = projection->closest_point;
    m_last_path_ref = projection->closest_point;
  }
  u = m_controller->control(vehicle_state, path, closest_point);
  m_startStop->triggerFSM(vehicle_state, u);

  for (auto& actuator : m_actuators)
    actuator.second->actuate(u);
}

};