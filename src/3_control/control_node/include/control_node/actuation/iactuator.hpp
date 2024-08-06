#ifndef CONTROLNODE_ACTUATION_ACTUATOR_HPP
#define CONTROLNODE_ACTUATION_ACTUATOR_HPP

#include <control_node/control/control.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/node.hpp>

namespace control_node {
namespace actuation {

struct IActuator {
  virtual ~IActuator() = default;
  virtual void init(rclcpp::Node& node, const Parameters& p) = 0;
  virtual void actuate(const control::Control& control) = 0;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP