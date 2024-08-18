#include <control_node/viz/viz_manager.hpp>
#include <control_node/parameters.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <algorithm>

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

  m_viz_mgr = &viz_mgr;

  auto viz_p = p.subparams("target_marker");
  double scale = viz_p.get<double>("diameter");
  m_viz_lookforward = viz_mgr.get_new(
    viz::msgs::Marker::CYLINDER,
    viz_p.parse_rgba("color", m_viz_lookforward_alpha),
    { scale, scale, 0.01 }
  );
}

void PurePursuit2023::viz(std::optional<Eigen::Vector2d> target) {
  if (m_viz_lookforward < 0)
    return;

  if (auto marker = m_viz_mgr->get_if_viz_tick(m_viz_lookforward))
  {
    if (target.has_value()) {
      SET_XY(marker->pose.position, target->x(), target->y());
      marker->color.a = m_viz_lookforward_alpha;
    } else {
      marker->color.a = 0.0f;
    }
  }
}

Control PurePursuit2023::control(
  std::chrono::nanoseconds,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
) {
  double lookforward = m_minLookForward;
  std::optional<Eigen::Vector2d> target;

  if (vehicle_path_projection.has_value() && vehicle_path_projection.has_value()) {
    auto target_ref = reference_path.advance_point(*vehicle_path_projection, lookforward);
    target = reference_path.get_position(target_ref);
  }

  viz(target);

  Control u(0.0, 0.1, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);

  if (state.position().has_value() && state.yaw().has_value() && target.has_value()) {
    u.steer = calculateSteeringTarget(
      *target,
      *state.position(),
      *state.yaw(),
      lookforward,
      m_steerGain,
      m_vp->max_steering_angle(),
      m_vp->lr(),
      m_vp->wheelbase()
    );
  }

  return u;
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node