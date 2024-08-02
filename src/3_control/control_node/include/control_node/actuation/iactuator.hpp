#ifndef CONTROLNODE_ACTUATION_ACTUATOR_HPP
#define CONTROLNODE_ACTUATION_ACTUATOR_HPP

#include <control_node/estimation/vehicle_state.hpp>
#include <control_node/control/control.hpp>
#include <rclcpp/node.hpp>

namespace control_node {
namespace actuation {

struct IActuator {
  virtual void actuate(const control::Control& control) = 0;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP