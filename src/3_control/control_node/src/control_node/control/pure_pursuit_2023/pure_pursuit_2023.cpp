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

static inline double curv_from_steer(double wheel_ang_rad, const VehicleParameters& vp) {
  double L = vp.wheelbase_m();
  double LR = vp.lr_m();
  double LF = L - vp.lr_m();

  double beta = std::atan(LR * std::tan(wheel_ang_rad) / L);
  double k = (std::tan(wheel_ang_rad) * std::cos(beta) - std::sin(beta)) / LF;
  return k;
}

double chord_len(double k, double theta) {
  return 2 * std::sin(theta / 2) / k;
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

  m_log_pub = node.create_publisher<mmr_base::msg::PurePursuitLog>(p.get<std::string>("log.topic"), p.parse_qos("log.qos_override"));

  m_logger = logger;
  m_minLookForward = p.get<double>("minLookForward");
  m_minLookForwardGain = p.get<double>("minLookForwardGain");
  m_speed_lookforward_gain = p.get<double>("speed_lookforward_gain");
  m_steerGain = p.get<double>("steerGain");
  m_minSpeedDistance = p.get<double>("minSpeedDistance");
  m_minSpeed = p.get<double>("minSpeed");
  m_simplified_longitudinal_control_enabled = p.get<bool>("low_level_longitudinal_controller.simplified");
  m_second_gear_on_second_lap = p.get<bool>("second_gear_on_second_lap");
  m_dynamic_change_gear = p.get<bool>("change_gear_logic");
  m_min_up = p.get<double>("m_min_up");
  m_max_up = p.get<double>("m_max_up");
  m_min_down = p.get<double>("m_min_down");
  m_max_down = p.get<double>("m_max_down");
  m_steer_delay_s = p.get<double>("steer_delay_s");
  m_max_accel_sq = p.get<double>("max_accel");
  m_max_accel_sq *= m_max_accel_sq;

  if (m_simplified_longitudinal_control_enabled) {
    m_simple_long_apps_p = p.get<double>("low_level_longitudinal_controller.apps_p");
    m_simple_long_brake_p = p.get<double>("low_level_longitudinal_controller.brake_p");
  }

  auto dyn_speed_p = p.subparams("dynamicTargetSpeed");
  m_dynamicTargetSpeed.enabled = dyn_speed_p.get<bool>("enabled");
  if (m_dynamicTargetSpeed.enabled) {
    m_dynamicTargetSpeed.slowLaps = dyn_speed_p.get<int>("slowLaps");
    m_dynamicTargetSpeed.maxSpeed = dyn_speed_p.get<double>("maxSpeed");
    m_dynamicTargetSpeed.targetSpeedWeight = dyn_speed_p.get<double>("targetSpeedWeight");
  }

  m_viz_mgr = &viz_mgr;

  auto viz_p = p.subparams("target_marker");
  double scale = viz_p.get<double>("diameter");
  m_viz_lookforward = viz_mgr.get_new(
    viz::msgs::Marker::CYLINDER,
    viz_p.parse_rgba("color", m_viz_lookforward_alpha),
    { scale, scale, 0.01 }
  );
}


int PurePursuit2023::gear_target(
  int acceleration_sign,
  const estimation::IVehicleState& state
  ) {
    double steering_angle = state.actual_steer().value();
    double ath = std::clamp<double>(state.throttle().value(), 0.0f, 1.0f);

    double rpm_up = lerp3(ath, mmr_point_double(0.0f, m_min_up), mmr_point_double(0.20f, m_min_up), mmr_point_double(1.0f, m_max_up));

    double rpm_down;

    if (state.gear().value() == 2) 
      rpm_down = 3500.0;
    else rpm_down = lerp3(ath, mmr_point_double(1.0f, m_min_down), mmr_point_double(0.70f, m_min_down), mmr_point_double(0.0f, m_max_down)); 

    if (acceleration_sign > 0 && state.rpm().value() >= rpm_up && state.gear().value() < 4) return state.gear().value() + 1;
    else if (acceleration_sign < 0 && state.rpm().value() <= rpm_down && state.gear().value() > 1 && std::abs(steering_angle) < 15.0) return state.gear().value() - 1;

    return state.gear().value();
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

template <typename T>
static inline int sign(T x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

Control PurePursuit2023::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap
) {
  // Compute the steer and speed lookforward.
  double steer_lookforward = m_minLookForward;
  double speed_lookforward = m_minSpeedDistance;
  if (state.speed().has_value()) {
    steer_lookforward += m_minLookForwardGain * state.speed().value();
    speed_lookforward += m_speed_lookforward_gain * state.speed().value();
  }
  
  // Do we know where we are on the track?
  bool is_projection_valid = vehicle_path_projection.has_value();

  // Get the steer target position
  std::optional<Eigen::Vector2d> targetPosition;
  if (is_projection_valid) {
    auto steer_target_ref = reference_path.advance_point(*vehicle_path_projection, steer_lookforward);
    targetPosition = reference_path.get_position(steer_target_ref);
  }

  // Compute the maximum speed
  double maximum_speed;
  if (m_dynamicTargetSpeed.enabled && lap > m_dynamicTargetSpeed.slowLaps) {
    // Use dynamic target speed

    if (!m_using_dynamic_speed) {
      RCLCPP_INFO(*this->m_logger, "Transitioning to dynamic target speed!");
      m_using_dynamic_speed = true;
    }

    // Get the target speed at the lookforward point
    if (is_projection_valid) {
      auto speed_target_ref = reference_path.advance_point(*vehicle_path_projection, speed_lookforward);

      if (auto max_speed_opt = reference_path.get_target_speed(speed_target_ref))
        maximum_speed = *max_speed_opt;
    }
  } else {
    // Use static speed
    maximum_speed = m_minSpeed;
  }

  viz(targetPosition);

  Control u(0.0, 0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);

  // Longitudinal control
  int accel_sign = 0;
  double target_acceleration = NAN;
  if (state.speed().has_value()) {
    if (m_simplified_longitudinal_control_enabled) {
      // Use a simple P control (useful for the simulator)
      double error = maximum_speed - state.speed().value();
      accel_sign = sign(error);
      u.throttle = m_simple_long_apps_p * std::max(error, 0.0);
      u.brake = m_simple_long_brake_p * std::max(-error, 0.0);
    } else {
      // Compute the target acceleration
      target_acceleration = (std::pow(maximum_speed, 2) - std::pow(*state.speed(), 2)) / (2 * speed_lookforward);

      accel_sign = sign(target_acceleration);

      // Apply maximum acceleration using GG diagram
      if (accel_sign > 0 && is_projection_valid) {
        auto k = reference_path.get_curvature(*vehicle_path_projection);
        if (k.has_value()) {
          double ay = k.value() * std::pow(state.speed().value(), 2.0);
          double ax_budget = std::sqrt(m_max_accel_sq - std::min(std::pow(ay, 2.0), m_max_accel_sq));
          target_acceleration = std::min(target_acceleration, ax_budget);
        }
      }

      // Determine the inputs from the low level controller
      if (state.gear().has_value() && state.rpm().has_value() && state.speed().has_value()) {
        AppsBrakePair ll_u = apps_brake_from_accel(target_acceleration, state.speed().value(), state.gear().value(), state.rpm().value(), *m_vp);

        u.throttle = ll_u.apps;
        u.brake = ll_u.brake_torque;
      }
    }
  }

  // Compute steer
  if (state.position().has_value() && state.yaw().has_value() && targetPosition.has_value()) {
    Eigen::Vector2d pred_target_position = *targetPosition;
    Eigen::Vector2d pred_car_position = *state.position();
    double pred_car_yaw = *state.yaw();

    // Predict the vehicle state to compensate for the actuator delay
    if (m_steer_delay_s > 0 && state.speed().has_value() && state.actual_steer().has_value()) {
      double actual_wheel_angle = state.actual_steer().value() / m_vp->steering_ratio();
      double k = curv_from_steer(actual_wheel_angle, *m_vp);

      double dtheta;
      double phi;
      double dist;
      double ds = state.speed().value() * m_steer_delay_s;

      if (std::abs(k) < 1e-7) {
        dtheta = 0;
        phi = 0;
        dist = ds;
      }
      else {
        dtheta = k * ds;
        phi = (std::numbers::pi - dtheta) / 2;
        phi = std::numbers::pi - phi;
        dist = chord_len(k, dtheta);
      }

      pred_car_position += dist * Eigen::Vector2d(
        std::cos(phi + pred_car_yaw),
        std::sin(phi + pred_car_yaw)
      );
      pred_car_yaw += dtheta;

      auto ref = reference_path.advance_point(*vehicle_path_projection, dist);
      pred_target_position = reference_path.get_position(ref);
    }

    double wheel_angle_rad = calculateSteeringTarget(
      pred_target_position,
      pred_car_position,
      pred_car_yaw,
      m_steerGain,
      m_vp->lr_m(),
      m_vp->wheelbase_m()
    );

    double wheel_angle_deg = wheel_angle_rad * (180 / std::numbers::pi);
    double steering_wheel_angle_deg = wheel_angle_deg * m_vp->steering_ratio();
    u.steer = steering_wheel_angle_deg;
  }

  // Compute target gear
  if (lap > 1 && m_second_gear_on_second_lap) {
    u.gear = 2;
  }
  else if (m_dynamic_change_gear) {
    u.gear = this->gear_target(accel_sign, state);
  }


  mmr_base::msg::PurePursuitLog log_msg;
  log_msg.header.frame_id = "ocropoid";
  log_msg.header.stamp = rclcpp::Time(t.count());
  log_msg.current_speed_m_s = state.speed().value_or(NAN);
  log_msg.steer_lookahead_m = steer_lookforward;
  log_msg.speed_lookahead_m = speed_lookforward;

  log_msg.raw_target_acceleration_m_s_2 = target_acceleration;
  log_msg.raw_target_speed_m_s = maximum_speed;

  log_msg.smoothed_target_speed_m_s = NAN;
  log_msg.smoothed_target_acceleration_m_s_2 = NAN;

  m_log_pub->publish(log_msg);

  return u;
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node