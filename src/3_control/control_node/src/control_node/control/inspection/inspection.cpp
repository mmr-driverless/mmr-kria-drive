#include <control_node/control/inspection/inspection.hpp>

namespace control_node {
namespace control {
namespace inspection {

Inspection::Inspection(const Parameters& p, const VehicleParameters& vp, rclcpp::Clock clock)
  : m_vp(vp),
    m_velocityMultiplier(p.get<float>("velocityMultiplier")),
    m_clock(clock)
{}

Control Inspection::control(
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
) {
  double steering = std::sin( m_velocityMultiplier * m_clock.now().nanoseconds()) * m_vp.max_steering_angle() * 0.5;

  // TODO: look ma! no throttle!
  return Control(steering, 0.0, 0.0, 0.0, 0, false);
}

}; // namespace inspection
}; // namespace control
}; // namespace control_node