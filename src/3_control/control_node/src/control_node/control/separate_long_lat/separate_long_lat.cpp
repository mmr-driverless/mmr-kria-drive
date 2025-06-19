#include <control_node/control/separate_long_lat/separate_long_lat.hpp>
#include <control_node/control/separate_long_lat/longitudinal_controller_factory.hpp>
#include <control_node/control/separate_long_lat/lateral_controller_factory.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

static inline int assert_valid_index(int idx, int n, rclcpp::Logger logger) {
  if (idx >= 0 && idx < n)
    return idx;

  RCLCPP_ERROR(logger, "Bad controller indices for longitudinal/lateral switch logic!");
  throw std::invalid_argument("Bad controller indices for longitudinal/lateral switch logic!");
}

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

  Parameters switch_p = p.subparams("switch");
  m_switch_lap = switch_p.get<int>("on_lap");
  m_lateral_controller_idx.first = assert_valid_index(switch_p.get<int>("lateral_before"), m_lateral_controllers.size(), logger);
  m_lateral_controller_idx.second = assert_valid_index(switch_p.get<int>("lateral_after"), m_lateral_controllers.size(), logger);
  m_longitudinal_controller_idx.first = assert_valid_index(switch_p.get<int>("longitudinal_before"), m_longitudinal_controllers.size(), logger);
  m_longitudinal_controller_idx.second = assert_valid_index(switch_p.get<int>("longitudinal_after"), m_longitudinal_controllers.size(), logger);
}

Control SeparateLongitudinalLateralController::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap
) {
  int long_idx;
  int lat_idx;

  if (lap < m_switch_lap) {
    lat_idx = m_lateral_controller_idx.first;
    long_idx = m_longitudinal_controller_idx.first;
  } else {
    lat_idx = m_lateral_controller_idx.second;
    long_idx = m_longitudinal_controller_idx.second;
  }

  LateralControl lat_ctrl = m_lateral_controllers[lat_idx].second->control(t, state, reference_path, vehicle_path_projection, lap);
  LongitudinalControl long_ctrl = m_longitudinal_controllers[long_idx].second->control(t, state, reference_path, vehicle_path_projection, lap);

  // Convert wheel angle to steering wheel angle
  if (m_convert_to_steering_wheel_degrees) {
    double wheel_angle_deg = lat_ctrl.steer * (180 / std::numbers::pi);
    double steering_wheel_angle_deg = wheel_angle_deg * m_vp->steering_ratio();

    lat_ctrl.steer = steering_wheel_angle_deg;
  } 

  return Control (
    lat_ctrl.steer,
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