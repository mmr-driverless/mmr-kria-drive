#ifndef CONTROLNODE_ACTUATION_ACTUATOR_HPP
#define CONTROLNODE_ACTUATION_ACTUATOR_HPP

#include <control_node/control/control.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/node.hpp>

namespace control_node {
namespace actuation {

struct IActuator {
  virtual ~IActuator() = default;
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) = 0;
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& control) = 0;

  virtual void request_enable() = 0;
  virtual void request_disable() = 0;
  virtual bool enabled() const = 0;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP