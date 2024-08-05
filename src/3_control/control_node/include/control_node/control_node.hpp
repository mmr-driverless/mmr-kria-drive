#ifndef CONTROLNODE_CONTROLNODE_HPP
#define CONTROLNODE_CONTROLNODE_HPP

#include "control_node/path/reference_path_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/control/icontroller.hpp>
#include <control_node/actuation/iactuator.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/viz_msgs.hpp>
#include <control_node/edf.hpp>

namespace control_node {

class ControlNode : public NodeBase
{
  std::chrono::milliseconds m_tick_interval;
  VehicleParameters m_vp;

  path::ReferencePathManager m_refpath_mgr;
  std::unique_ptr<estimation::IStateEstimator> m_estimator;
  std::unique_ptr<control::IController> m_controller;
  std::vector<std::unique_ptr<actuation::IActuator>> m_actuators;

  std::optional<path::ReferencePath::PointRef> m_last_path_ref;

  rclcpp::Publisher<viz_msgs::MarkerArray>::SharedPtr m_viz_pub;

  bool m_has_completed_path;

public:
  ControlNode();
  void tick();

  inline std::chrono::milliseconds tick_interval() const { return m_tick_interval; }
};

};

#endif // !CONTROLNODE_CONTROLNODE_HPP