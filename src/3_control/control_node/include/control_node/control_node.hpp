#ifndef CONTROLNODE_CONTROLNODE_HPP
#define CONTROLNODE_CONTROLNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mmr_kria_base/msg/marker.hpp>
#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/control/icontroller.hpp>
#include <control_node/actuation/iactuator.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node {

class ControlNode : public rclcpp::Node {
  std::chrono::milliseconds m_tick_interval;
  VehicleParameters m_vp;

  std::unique_ptr<estimation::IStateEstimator> m_estimator;
  std::unique_ptr<control::IController> m_controller;
  std::vector<std::unique_ptr<actuation::IActuator>> m_actuators;

  std::optional<path::ReferencePath::PointRef> m_last_path_ref;
  path::ReferencePath m_path;

  std::vector<Eigen::Vector2d> m_waypoints;
  std::vector<path::ReferencePath::PointData::StorageT> m_path_data;

public:
  ControlNode();
  void tick();

  inline std::chrono::milliseconds tick_interval() const { return m_tick_interval; }
  void center_line_cb(mmr_kria_base::msg::Marker::SharedPtr msg);
  void center_line_completed_cb(mmr_kria_base::msg::Marker::SharedPtr msg);
};

};

#endif // !CONTROLNODE_CONTROLNODE_HPP