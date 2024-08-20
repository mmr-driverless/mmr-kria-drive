#include <control_node/control/inspection/inspection.hpp>

namespace control_node {
namespace control {
namespace inspection {

void Inspection::init(rclcpp::Node&, const Parameters& p, const VehicleParameters& vp, viz::VizManager&, rclcpp::Logger) {
  m_vp = &vp;
  m_frequency = p.get<double>("frequency");
  m_steer_fraction = p.get<double>("steer_fraction");
  m_throttle = p.get<double>("throttle");
  m_gear = p.get<int>("gear");
  m_lc = (p.get<bool>("launch_control")? Control::LaunchControl::Set : Control::LaunchControl::Unset);
}

Control Inspection::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState&,
  const path::ReferencePath&,
  const std::optional<path::ReferencePath::PointRef>&,
  int
) {
  if (!m_start_t.has_value())
    m_start_t = t;
    
  double steering = std::sin(m_frequency * std::chrono::duration<double>(t - *m_start_t).count()) * m_vp->max_steering_angle_rad() * m_steer_fraction;
  return Control(steering, m_throttle, 0.0, Control::Clutch::Engaged, m_gear, m_lc);
}

}; // namespace inspection
}; // namespace control
}; // namespace control_node