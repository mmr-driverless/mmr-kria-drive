#include "control_node/control/separate_long_lat/lateral_control.hpp"
#include <control_node/control/separate_long_lat/lateral/pure_pursuit/pure_pursuit.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {
namespace lateral{
namespace pure_pursuit{

static inline double normalizeAngle(double angle){
  while(angle > M_PI) angle -= (2 * M_PI);
  while(angle < -M_PI) angle += (2 * M_PI);
  return angle;
}

void PurePursuit::viz(std::optional<Eigen::Vector2d> target) {
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

template <typename T>
static inline int sign(T x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

static inline double calculateSteeringTarget(Eigen::Vector2d target, Eigen::Vector2d car_position, double car_yaw, double steer_gain, double com_dist_to_rear, double wheelbase)
{
  Eigen::Vector2d car_rear = car_position - com_dist_to_rear * Eigen::Vector2d(std::cos(car_yaw), std::sin(car_yaw));
  Eigen::Vector2d diff = target - car_rear;

  //Calculate delta between target direction and car Rotation
  double SteerTarget = normalizeAngle(std::atan2(diff.y(), diff.x()) - car_yaw);

  //Calculate steer target rotation
  double dist = diff.norm();
  double wheelRotation = std::atan2(2 * wheelbase * std::sin(SteerTarget) / (dist * steer_gain), 1);
  
  return wheelRotation;
}

void PurePursuit::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) 
{
  m_vp = &vp;

  m_log_pub = node.create_publisher<mmr_base::msg::PurePursuitLog>(p.get<std::string>("log.topic"), p.parse_qos("log.qos_override"));

  m_logger = logger;
  m_minLookForward = p.get<double>("minLookForward");
  m_minLookForwardGain = p.get<double>("minLookForwardGain");
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

LateralControl PurePursuit::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap
) {

  (void) lap;

  // Compute the steer lookforward.
  double steer_lookforward = m_minLookForward;
  if (state.speed().has_value()) {
    steer_lookforward += m_minLookForwardGain * state.speed().value();
  }
  
  // Do we know where we are on the track?
  bool is_projection_valid = vehicle_path_projection.has_value();

  // Get the steer target position
  std::optional<Eigen::Vector2d> targetPosition;
  if (is_projection_valid) {
    auto steer_target_ref = reference_path.advance_point(*vehicle_path_projection, steer_lookforward);
    targetPosition = reference_path.get_position(steer_target_ref);
  }

  viz(targetPosition);

  LateralControl u(0.0);

  // Compute steer
  if (state.position().has_value() && state.yaw().has_value() && targetPosition.has_value()) {
    double wheel_angle_rad = calculateSteeringTarget(
      *targetPosition,
      *state.position(),
      *state.yaw(),
      m_steerGain,
      m_vp->lr_m(),
      m_vp->wheelbase_m()
    );

    double wheel_angle_deg = wheel_angle_rad * (180 / std::numbers::pi);
    double steering_wheel_angle_deg = wheel_angle_deg * m_vp->steering_ratio();
    u.steer = steering_wheel_angle_deg;
  }

  mmr_base::msg::PurePursuitLog log_msg;
  log_msg.header.frame_id = "ocropoid"; // lol
  log_msg.header.stamp = rclcpp::Time(t.count());
  log_msg.current_speed_m_s = state.speed().value_or(NAN);
  log_msg.steer_lookahead_m = steer_lookforward;

  log_msg.smoothed_target_speed_m_s = NAN;
  log_msg.smoothed_target_acceleration_m_s_2 = NAN;

  m_log_pub->publish(log_msg);

  return u;
}


}// namespace pure_pursuit
}// namespace lateral
}// namespace separate_long_lat
}// namespace control
}// namespace control_node
