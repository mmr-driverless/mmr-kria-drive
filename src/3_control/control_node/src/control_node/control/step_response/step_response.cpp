#include <control_node/control/step_response/step_response.hpp>
#include <control_node/control/pure_pursuit_2023/acc_to_brake_apps.hpp>

namespace control_node {
namespace control {
namespace step_response{


void StepResponse::init(rclcpp::Node&, const Parameters& p, const VehicleParameters& vp, rclcpp::Logger) {
  m_vp = &vp;
  m_acc_target = p.get<double>("acc_target");
  m_gear_target = p.get<int>("gear_target");
  m_min_throttle = p.get<float>("min_throttle");
  m_use_step = false;
}

Control StepResponse::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath&,
  const std::optional<path::ReferencePath::PointRef>&
) {

  double throttle = 0.0;

  if(state.bag_button().has_value() && (state.bag_button().value() == true))
  {
    m_use_step = true;
  }
  if(state.speed().has_value() && state.gear().has_value() && state.rpm().has_value())
  {
    if(m_use_step)
    {
      throttle =  apps_brake_from_accel(m_acc_target, state.speed().value(), state.gear().value(), state.rpm().value(), vp);
    }
    else
    {
      throttle =  apps_brake_from_accel(0.0, state.speed().value(), state.gear().value(), state.rpm().value(), vp);
    }
  }

  double ecu_throttle = m_min_throttle + (1 - m_min_throttle) * throttle;

  return Control(0.0, ecu_throttle, 0.0, Control::Clutch::Engaged, m_gear_target, Control::LaunchControl::Unset);
}

}; // namespace step_response
}; // namespace control
}; // namespace control_node