#ifndef CONTROLNODE_CONTROLNODE_HPP
#define CONTROLNODE_CONTROLNODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <control_node/actuation/actuator_manager.hpp>
#include <control_node/viz/viz_manager.hpp>
#include <control_node/path/reference_path_manager.hpp>
#include <control_node/event/event_manager.hpp>
#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>

#include <control_node/edf.hpp>

namespace control_node {

class ControlNode : public NodeBase
{
  std::chrono::milliseconds m_tick_interval;
  VehicleParameters m_vp;

  actuation::ActuatorManager m_actuator_mgr;
  event::EventManager m_event_mgr;
  path::ReferencePathManager m_refpath_mgr;
  
  viz::VizManager m_viz_mgr;
  
  int m_path_projection_marker;
  int m_path_threshold_marker;
  int m_path_search_start_marker;
  int m_path_search_end_marker;

  float m_path_projection_marker_alpha;
  float m_path_threshold_marker_alpha;
  float m_path_search_start_marker_alpha;
  float m_path_search_end_marker_alpha;

  std::unique_ptr<estimation::IStateEstimator> m_estimator;
  std::unique_ptr<control::IController> m_controller;

  double m_path_threshold2;

  std::optional<path::ReferencePath::PointRef> m_last_path_ref;

  bool m_has_completed_path;

  void setup_estimator();
  void setup_controller();

public:
  ControlNode();
  void tick();

  inline std::chrono::milliseconds tick_interval() const { return m_tick_interval; }
};

};

#endif // !CONTROLNODE_CONTROLNODE_HPP