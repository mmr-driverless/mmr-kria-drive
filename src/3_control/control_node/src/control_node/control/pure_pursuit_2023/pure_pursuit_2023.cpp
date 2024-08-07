#include <control_node/parameters.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
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

void PurePursuit2023::init(rclcpp::Node&, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr) {
  m_vp = &vp;
  m_minLookForward = p.get<double>("minLookForward");
  m_steerGain = p.get<double>("steerGain");
  
  Parameters p_color = p.subparams("target_viz_color");

  m_viz_lookforward_alpha = p_color.get<double>("a");
  m_viz_lookforward = viz_mgr.get_new(
    viz::msgs::Marker::CYLINDER,
    p_color.get<double>("r"),
    p_color.get<double>("g"),
    p_color.get<double>("b"),
    0.0
  );
}

Control PurePursuit2023::control(
  std::chrono::nanoseconds,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
) {
  double lookforward = m_minLookForward;

  if (!vehicle_path_projection.has_value()) {
    // TODO: Choose a better safe state. Putting it in 1st no matter the speed is most likely not a good idea.
    m_viz_lookforward->color.a = 0;
    return Control(0.0, 0.0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);
  }

  auto target_ref = reference_path.advance_point(*vehicle_path_projection, lookforward);
  Eigen::Vector2d target = reference_path.get_position(target_ref);
  
  SET_XY(m_viz_lookforward->pose.position, target.x(), target.y());
  m_viz_lookforward->color.a = m_viz_lookforward_alpha;

  double steering = calculateSteeringTarget(
    target,
    state.position(),
    state.yaw(),
    lookforward,
    m_steerGain,
    m_vp->max_steering_angle(),
    m_vp->lr(),
    m_vp->wheelbase()
  );

  // TODO: look ma! no throttle!
  return Control(steering, 0.0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node