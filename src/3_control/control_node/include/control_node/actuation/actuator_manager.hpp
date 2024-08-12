#ifndef CONTROLNODE_ACTUATION_ACTUATORMANAGER_HPP
#define CONTROLNODE_ACTUATION_ACTUATORMANAGER_HPP

#include <algorithm>
#include <rclcpp/node.hpp>
#include <vector>

#include <control_node/actuation/actuator_factory.hpp>
#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace actuation {

class ActuatorManager {
  std::vector<std::pair<int, std::unique_ptr<actuation::IActuator>>> m_actuators;
  rclcpp::Logger m_logger;

public:
  ActuatorManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger);

  void request_enable_all() const;

  void request_disable_all() const;

  bool all_enabled() const;

  void actuate_all(const control::Control &u) const;
};

}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_ACTUATORMANAGER_HPP