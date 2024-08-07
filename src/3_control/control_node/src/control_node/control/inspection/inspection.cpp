#include <control_node/control/inspection/inspection.hpp>

namespace control_node {
namespace control {
namespace inspection {

Inspection::Inspection()
  : m_vp(nullptr),
    m_frequency(0.0),
    m_steer_fraction(0.0)
{ }

void Inspection::init(rclcpp::Node&, const Parameters& p, const VehicleParameters& vp, viz::VizManager&) {
  m_vp = &vp;
  m_frequency = p.get<double>("frequency");
  m_steer_fraction = p.get<double>("steer_fraction");
}

Control Inspection::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState&,
  const path::ReferencePath&,
  const std::optional<path::ReferencePath::PointRef>&
) {
  if (!m_start_t.has_value())
    m_start_t = t;

  double steering = std::sin(m_frequency * std::chrono::duration<double>(t - *m_start_t).count()) * m_vp->max_steering_angle() * m_steer_fraction;
  return Control(steering, 0.0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);
}

}; // namespace inspection
}; // namespace control
}; // namespace control_node