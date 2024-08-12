#include <control_node/actuation/actuator_manager.hpp>

namespace control_node {
namespace actuation {

ActuatorManager::ActuatorManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger)
  : m_logger(logger)
{
  m_actuators = actuation::get_factory().from_param_list(p, "actuators", [&node, &logger](actuation::IActuator& act, int i, const Parameters& p_i, const std::string& type) {
    act.init(*node, p_i, logger.get_child(type + "(" + std::to_string(i) + ")"));
  }, logger);

  if (m_actuators.size() == 0)
    RCLCPP_WARN(logger, "NO actuators initialized!");
  else
    RCLCPP_INFO(logger, "INITIALIZED %zu actuators.", m_actuators.size());
}

void ActuatorManager::request_enable_all() const {
  RCLCPP_INFO(m_logger, "Requesting all actuators to ENABLE!");

  for (const auto& act : m_actuators)
    act.second->request_enable();
}

void ActuatorManager::request_disable_all() const {
  RCLCPP_INFO(m_logger, "Requesting all actuators to DISABLE!");

  for (const auto& act : m_actuators)
    act.second->request_disable();
}

bool ActuatorManager::all_enabled() const {
  return std::all_of(m_actuators.begin(), m_actuators.end(), [](const auto& act) { return act.second->enabled(); });
}

void ActuatorManager::actuate_all(const control::Control &u) const {
  for (const auto& act : m_actuators)
    act.second->actuate(u);
}

}; // namespace actuation
}; // namespace control_node