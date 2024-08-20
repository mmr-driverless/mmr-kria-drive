#include <chrono>
#include <rclcpp/logging.hpp>
#include <stdexcept>

#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/parameters.hpp>
#include <control_node/control_node.hpp>

#include <control_node/path/reference_path.hpp>

#include <control_node/actuation/actuator_factory.hpp>
#include <control_node/estimation/state_estimator_factory.hpp>
#include <control_node/control/controller_factory.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/viz/viz_manager.hpp>
using namespace std::chrono_literals;

namespace control_node {

inline static void update_marker(viz::VizManager& mgr, int mid, std::optional<Eigen::Vector2d> position, float alpha, double z) {
  if (mid < 0)
    return;

  auto& marker = mgr.get(mid);

  if (position.has_value()) {
    SET_XYZ(marker.pose.position, position->x(), position->y(), z);
    marker.color.a = alpha;
  } else {
    marker.color.a = 0.0f;
  }
}

void ControlNode::tick() {
  std::chrono::nanoseconds t(this->get_clock()->now().nanoseconds());
  RCLCPP_DEBUG(this->get_logger(), "Tick @%ld.%03lds", std::chrono::duration_cast<std::chrono::seconds>(t).count(), std::chrono::duration_cast<std::chrono::milliseconds>(t).count() % 1000);

  m_viz_mgr.pre_tick(t);

  // Estimate the current state.
  const estimation::IVehicleState& x = m_estimator->update_and_get_current_state();

  // Retrieve the current path. If it changed, invalidate the last path reference.
  path::ReferencePath path = m_refpath_mgr.get();
  if (m_refpath_mgr.changed())
    m_last_path_ref = std::nullopt;

  std::optional<Eigen::Vector2d> search_start_point;
  std::optional<Eigen::Vector2d> search_end_point;
  std::optional<path::ReferencePath::PointRef> closest_point;
  if (x.position().has_value()) {
    // Project the vehicle onto the path.
    auto projection = path.project_vehicle(*x.position(), m_last_path_ref, m_path_threshold2);

    // If the projection succeeds, store the closest point and update the last path reference.
    if (projection.has_value()) {
      search_start_point = path.get_position(projection->first_checked_waypoint);
      search_end_point = path.get_position(projection->last_checked_waypoint);
      closest_point = projection->closest_point;
      m_last_path_ref = projection->closest_point;
    }
  }

  // Decide what inputs to apply based on the current vehicle state and position relative to the path.
  control::Control u = m_controller->control(t, x, path, closest_point, m_event_mgr.lap());

  // Override the controls to perform the start and stop maneuvers.
  u = m_event_mgr.tick(t, x, u);

  // Actuate the control input.
  m_actuator_mgr.actuate_all(t, u);

  if (m_viz_mgr.is_viz_tick()) {
    update_marker(m_viz_mgr, m_path_projection_marker, closest_point.has_value()? std::make_optional(path.get_position(*closest_point)) : std::nullopt, m_path_projection_marker_alpha, 0);
    update_marker(m_viz_mgr, m_path_threshold_marker, x.position(), m_path_threshold_marker_alpha, -0.1);
    update_marker(m_viz_mgr, m_path_search_start_marker, search_start_point, m_path_search_start_marker_alpha, -0.1);
    update_marker(m_viz_mgr, m_path_search_end_marker, search_end_point, m_path_search_end_marker_alpha, -0.1);
  }

  // Update the visualization.
  m_viz_mgr.tick();
}


inline static std::array<double, 3> flat_scale(double scale) {
  return { scale, scale, 0.01 };
}

ControlNode::ControlNode() : NodeBase("control_node"),
  m_tick_interval(std::chrono::milliseconds(Parameters(this).get<int>("tick_interval"))),
  m_vp(VehicleParameters(Parameters(this, "vehicle_parameters"))),
  m_actuator_mgr(this, Parameters(this, "actuation"), this->get_logger().get_child("ActuatorMgr")),
  m_event_mgr(this, Parameters(this, "event_manager"), this->get_logger().get_child("EventMgr"), m_actuator_mgr),
  m_refpath_mgr(this, Parameters(this, "reference_path_manager"), this->get_logger().get_child("RefPathMgr")),
  m_viz_mgr(this, Parameters(this, "viz"), this->get_logger().get_child("VizMgr")) 
{
  #ifdef USE_EDF
  this->configureEDFScheduler(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_interval()).count(),
    this->declare_parameter("wcet_ns", rclcpp::PARAMETER_INTEGER).get<int>(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(tick_interval()).count()
  );
  #endif

  auto tracking_p = Parameters(this, "tracking");
  double path_threshold = tracking_p.get<double>("threshold");
  m_path_threshold2 = path_threshold * path_threshold;

  m_has_completed_path = false;

  auto projection_marker_p = tracking_p.subparams("projection_marker");
  m_path_projection_marker = m_viz_mgr.get_new(viz::msgs::Marker::CYLINDER, projection_marker_p.parse_rgba("color", m_path_projection_marker_alpha), flat_scale(projection_marker_p.get<double>("diameter")));

  auto threshold_marker_p = tracking_p.subparams("threshold_marker");
  m_path_threshold_marker = m_viz_mgr.get_new(viz::msgs::Marker::CYLINDER, threshold_marker_p.parse_rgba("color", m_path_threshold_marker_alpha), flat_scale(path_threshold * 2));

  auto search_start_marker_p = tracking_p.subparams("search_start_marker");
  m_path_search_start_marker = m_viz_mgr.get_new(viz::msgs::Marker::CUBE, search_start_marker_p.parse_rgba("color", m_path_search_start_marker_alpha), flat_scale(search_start_marker_p.get<double>("length")));

  auto search_end_marker_p = tracking_p.subparams("search_end_marker");
  m_path_search_end_marker = m_viz_mgr.get_new(viz::msgs::Marker::CUBE, search_end_marker_p.parse_rgba("color", m_path_search_end_marker_alpha), flat_scale(search_end_marker_p.get<double>("length")));

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
  m_controller->init(*this, p.subparams("params"), m_vp, m_viz_mgr, this->get_logger().get_child(type));
}

};