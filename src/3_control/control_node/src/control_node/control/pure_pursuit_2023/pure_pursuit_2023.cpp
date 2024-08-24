#include <cmath>
#include <control_node/viz/viz_manager.hpp>
#include <control_node/parameters.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/control/pure_pursuit_2023/acc_to_brake_apps.hpp>
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

void PurePursuit2023::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) {
  m_vp = &vp;

  m_targetSpeedPub = node.create_publisher<sensor_msgs::msg::Temperature>("/control/targetSpeed", 1);

  m_logger = logger;
  m_minLookForward = p.get<double>("minLookForward");
  m_minLookForwardGain = p.get<double>("minLookForwardGain");
  m_steerGain = p.get<double>("steerGain");
  m_minSpeedDistance = p.get<double>("minSpeedDistance");
  m_minSpeed = p.get<double>("minSpeed");
  m_min_throttle = p.get<double>("min_throttle");
  m_simplified_longitudinal_control_enabled = p.get<bool>("low_level_longitudinal_controller.simplified");
  m_second_gear_on_second_lap = p.get<bool>("second_gear_on_second_lap");

  if (m_simplified_longitudinal_control_enabled) {
    m_simple_long_apps_p = p.get<double>("low_level_longitudinal_controller.apps_p");
    m_simple_long_brake_p = p.get<double>("low_level_longitudinal_controller.brake_p");
  } else {
    m_ll_accel_lookforward = p.get<double>("low_level_longitudinal_controller.accel_lookforward");
    m_ll_accel_k_smooth = p.get<double>("low_level_longitudinal_controller.accel_k_smooth");
  }

  auto dyn_speed_p = p.subparams("dynamicTargetSpeed");
  m_dynamicTargetSpeed.enabled = dyn_speed_p.get<bool>("enabled");
  if (m_dynamicTargetSpeed.enabled) {
    m_dynamicTargetSpeed.slowLaps = dyn_speed_p.get<int>("slowLaps");
    m_dynamicTargetSpeed.k_smooth = dyn_speed_p.get<double>("k_smooth");
    m_dynamicTargetSpeed.maxSpeed = dyn_speed_p.get<double>("maxSpeed");
    m_dynamicTargetSpeed.targetSpeedWeight = dyn_speed_p.get<double>("targetSpeedWeight");
  }

  m_smoothedSpeed = m_minSpeed;

  m_viz_mgr = &viz_mgr;

  auto viz_p = p.subparams("target_marker");
  double scale = viz_p.get<double>("diameter");
  m_viz_lookforward = viz_mgr.get_new(
    viz::msgs::Marker::CYLINDER,
    viz_p.parse_rgba("color", m_viz_lookforward_alpha),
    { scale, scale, 0.01 }
  );
}

void PurePursuit2023::pub_target_speed(std::chrono::nanoseconds t, double speed) {
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = rclcpp::Time(t.count());
  msg.temperature = speed;
  m_targetSpeedPub->publish(msg);
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
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap
) {

  // If we have a speed estimate, compute the dynamic lookforward
  double dynamic_lookforward = 0.0;
  if (state.speed().has_value())
    dynamic_lookforward = m_minLookForwardGain * state.speed().value();
  
  // Compute the steer and speed lookforward.
  double steer_lookforward = m_minLookForward + dynamic_lookforward;
  double speed_lookforward = m_minSpeedDistance + dynamic_lookforward;

  // Do we know where we are on the track?
  bool is_projection_valid = vehicle_path_projection.has_value();

  // Get the steer target position
  std::optional<Eigen::Vector2d> targetPosition;
  if (is_projection_valid) {
    auto steer_target_ref = reference_path.advance_point(*vehicle_path_projection, steer_lookforward);
    targetPosition = reference_path.get_position(steer_target_ref);
  }

  // Compute the target speed
  double targetSpeed;
  if (m_dynamicTargetSpeed.enabled && lap > m_dynamicTargetSpeed.slowLaps) {
    // Use dynamic target speed

    if (!m_using_dynamic_speed) {
      RCLCPP_INFO(*this->m_logger, "Transitioning to dynamic target speed!");
      m_using_dynamic_speed = true;
    }

    // Get the target speed at the lookforward point
    double new_target_speed = m_minSpeed;
    if (is_projection_valid) {
      auto speed_target_ref = reference_path.advance_point(*vehicle_path_projection, speed_lookforward);

      if (auto tgt_speed = reference_path.get_target_speed(speed_target_ref))
        new_target_speed = *tgt_speed;
    }

    // Smooth the target speed in the time-domain with a first order IIR filter
    double smoothed = m_smoothedSpeed * (1 - m_dynamicTargetSpeed.k_smooth) + new_target_speed * m_dynamicTargetSpeed.k_smooth;
    if (std::isnan(smoothed)) {
      RCLCPP_ERROR(*m_logger, "NAN target speed!!!");
      smoothed = m_minSpeed;
    }

    m_smoothedSpeed = std::clamp<double>(smoothed, m_minSpeed, m_dynamicTargetSpeed.maxSpeed);
    targetSpeed = m_smoothedSpeed;
  } else {
    // Use static speed
    targetSpeed = m_minSpeed;
  }

  pub_target_speed(t, targetSpeed);
  viz(targetPosition);

  Control u(0.0, 0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);

  if (state.speed().has_value()) {
    if (m_simplified_longitudinal_control_enabled) {
      double error = targetSpeed - state.speed().value();
      u.throttle = m_simple_long_apps_p * std::max(error, 0.0);
      u.brake = m_simple_long_brake_p * std::max(-error, 0.0);
    } else {
      // Compute the target acceleration
      double accv = (std::pow(targetSpeed, 2) - std::pow(*state.speed(), 2)) / (2 * m_ll_accel_lookforward);

      // Smooth the target acceleration in the time-domain with yet another first order IIR filter
      m_smoothedAccel = m_smoothedAccel * (1 - m_ll_accel_k_smooth) + accv * m_ll_accel_k_smooth;

      // Determine the inputs from the low level controller
      if (state.gear().has_value() && state.rpm().has_value() && state.speed().has_value()) {
        AppsBrakePair ll_u = apps_brake_from_accel(accv, state.speed().value(), state.gear().value(), state.rpm().value(), *m_vp);

        u.throttle = ll_u.apps;
        u.brake = ll_u.brake_torque;
      }
    }
  }

  // Apply minimum throttle
  u.throttle = std::max(u.throttle, m_min_throttle);

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

  if (lap > 1 && m_second_gear_on_second_lap) {
    u.gear = 2;
  }

  return u;
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node