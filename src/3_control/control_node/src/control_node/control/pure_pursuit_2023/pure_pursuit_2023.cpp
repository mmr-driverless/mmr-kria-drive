#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

// Copy paste from old node, it's not my fault
// soz
static inline double normalizeAngle(double angle){
  while(angle > M_PI) angle -= (2 * M_PI);
  while(angle < -M_PI) angle += (2 * M_PI);
  return angle;
}

static inline double calculateSteeringTarget(Eigen::Vector2d target, Eigen::Vector2d car_position, double car_yaw, double lookforward, double steer_gain, double max_steer, double com_dist_to_rear, double wheelbase)
{
  Eigen::Vector2d car_rear = car_position - com_dist_to_rear * Eigen::Vector2d(std::cos(car_yaw), std::sin(car_yaw));

  //Calculate delta between target direction and car Rotation
  double SteerTarget = normalizeAngle(atan2(target.y() - car_rear.y(), target.x() - car_rear.x()) - car_yaw);

  //Calculate steer target rotation
  double wheelRotation = atan2(2 * wheelbase * std::sin(SteerTarget) / (lookforward * steer_gain), 1);

  //Cut off with respect to real steer car capability
  return std::clamp(wheelRotation, -max_steer, max_steer);
}

void PurePursuit2023::init(rclcpp::Node&, const Parameters& p, const VehicleParameters& vp) {
  m_vp = &vp;
  m_minLookForward = p.get<double>("minLookForward");
  m_steerGain = p.get<double>("steerGain");
}

Control PurePursuit2023::control(
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
) {
  double lookforward = m_minLookForward;

  if (!vehicle_path_projection.has_value()) {
    // TODO: Choose better safe state
    return Control(0.0, 0.0, 0.0, 0.0, 0, false);
  }

  auto target_ref = reference_path.advance_point(*vehicle_path_projection, lookforward);

  double steering = calculateSteeringTarget(
    reference_path.get_position(target_ref),
    state.position(),
    state.yaw(),
    lookforward,
    m_steerGain,
    m_vp->max_steering_angle(),
    m_vp->lr(),
    m_vp->wheelbase()
  );

  // TODO: look ma! no throttle!
  return Control(steering, 0.0, 0.0, 0.0, 0, false);
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node