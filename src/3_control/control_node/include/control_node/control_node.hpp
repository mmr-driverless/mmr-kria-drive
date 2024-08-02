#ifndef CONTROLNODE_CONTROLNODE_HPP
#define CONTROLNODE_CONTROLNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/control/icontroller.hpp>
#include <control_node/actuation/iactuator.hpp>
#include <control_node/path/reference_path.hpp>

namespace control_node {

class ControlNode : public rclcpp::Node {
  std::unique_ptr<estimation::IStateEstimator> m_estimator;
  std::unique_ptr<control::IController> m_controller;
  std::vector<std::unique_ptr<actuation::IActuator>> m_actuators;

  std::chrono::milliseconds m_tick_interval;

public:
  ControlNode();
  void tick();

  inline std::chrono::milliseconds tick_interval() const { return m_tick_interval; }
};

};

#endif // !CONTROLNODE_CONTROLNODE_HPP