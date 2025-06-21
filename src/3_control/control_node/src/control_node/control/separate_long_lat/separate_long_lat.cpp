#include <control_node/control/separate_long_lat/separate_long_lat.hpp>
#include <control_node/control/separate_long_lat/longitudinal_controller_factory.hpp>
#include <control_node/control/separate_long_lat/lateral_controller_factory.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {


void SeparateLongitudinalLateralController::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) {
  m_vp = &vp;

  m_longitudinal_controllers = get_longitudinal_controller_factory().from_param_list(p, "longitudinal", [&](ILongitudinalController& ctrl, int idx, const Parameters& p_i, const std::string& name) {
    (void)name;
    (void)idx;
    ctrl.init(node, p_i, vp, viz_mgr, logger);
  }, logger.get_child("ComponentFactory"));

  m_lateral_controllers = get_lateral_controller_factory().from_param_list(p, "lateral", [&](ILateralController& ctrl, int idx, const Parameters& p_i, const std::string& name) {
    (void)name;
    (void)idx;
    ctrl.init(node, p_i, vp, viz_mgr, logger);
  }, logger.get_child("ComponentFactory"));

  m_convert_to_steering_wheel_degrees = p.get<bool>("convert_to_steering_wheel_degrees");

  if(m_lateral_controllers.size() == 0 || m_longitudinal_controllers.size() == 0)
  {
    throw std::runtime_error("Must define at least one lateral controller and one longitudinal controller");
  }
  if(m_lateral_controllers.size() > 2)
  {
    throw std::runtime_error("Lateral controllers must at most be 2");
  }
  if(m_longitudinal_controllers.size() > 1)
  {
    throw std::runtime_error("Longitudinal controllers must be exactly 1");
  }
}

Control SeparateLongitudinalLateralController::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap
) {

  // std::cerr << "reference_path.is_from_global_planner(): " << reference_path.is_from_global_planner() << ", m_lateral_controllers.size(): " << m_lateral_controllers.size() << std::endl; 

  if (reference_path.is_from_global_planner() && (m_lateral_controllers.size() == 2)) // if we have recieved the path from the global planner and the second lateral controller exists we can switch to it
  {
    m_lat_idx = 1;
  }

  std::optional<LateralControl> lat_ctrl= m_lateral_controllers[m_lat_idx].second->control(t, state, reference_path, vehicle_path_projection, lap);
  LongitudinalControl long_ctrl = m_longitudinal_controllers[m_long_idx].second->control(t, state, reference_path, vehicle_path_projection, lap, lat_ctrl);

  // Convert wheel angle to steering wheel angle
  if (m_convert_to_steering_wheel_degrees && lat_ctrl.has_value()) {
    double wheel_angle_deg = lat_ctrl.value().steer * (180 / std::numbers::pi);
    double steering_wheel_angle_deg = wheel_angle_deg * m_vp->steering_ratio();

    lat_ctrl.value().steer = steering_wheel_angle_deg;
  } 

  if(lat_ctrl.has_value())
  {
    return Control (
      lat_ctrl.value().steer,
      long_ctrl.throttle,
      long_ctrl.brake,
      long_ctrl.clutch,
      long_ctrl.gear,
      long_ctrl.launch
  );
  }

  return Control (
    0.0,
    long_ctrl.throttle,
    long_ctrl.brake,
    long_ctrl.clutch,
    long_ctrl.gear,
    long_ctrl.launch
  );
}

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node