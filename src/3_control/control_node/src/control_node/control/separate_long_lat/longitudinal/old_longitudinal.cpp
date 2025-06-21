#include <control_node/control/separate_long_lat/longitudinal/old_longitudinal/old_longitudinal.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {
namespace longitudinal {
namespace old_longitudinal {

struct AppsBrakePair {
  double brake_torque;
  double apps;
};

template <typename RangeTY, typename RangeTX>
static inline double interp1d(const RangeTY& y, const RangeTX& x, double xq) {
  /*
    Evaluate f(xq) by linear interpolation, where f(x(i)) = y(i) for i=1..N
    Extrapolation is performed by taking the nearest f(x)
  */

  int N = y.size();
  assert((int)x.size() == N);

  if (xq <= x[0])
    return y[0];

  for (int i = 0; i < N - 1; ++i) {
    if (x[i] <= xq && xq < x[i+1]) {
      double t = xq - x[i];
      double m = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
      double q = y[i];
      return m * t + q;
    }
  }

  return y[N-1];
}

static inline auto apps_map_x_from_rpm(int rpm_act, const VehicleParameters& vp) {
  int N = vp.apps_map_rpm().size();

  if (rpm_act <= vp.apps_map_rpm()[0])
    return vp.apps_map_x().col(0);

  for (int i = 0; i < N - 1; ++i) {
    if (rpm_act >= vp.apps_map_rpm()[i] && rpm_act < vp.apps_map_rpm()[i + 1])
      return vp.apps_map_x().col(i);
  }

  return vp.apps_map_x().col(N-1);
}


static inline double apps_from_engine_torque(double torque_req, int rpm_act, const VehicleParameters& vp) {
  // Compute the maximum torque of the engine from the current RPM
  double max_torque = interp1d(vp.CDC_Nm(), vp.NMOVET_rpm(), (double)rpm_act);

  // Compute the required Torque% wrt maximum torque
  double torque_perc = torque_req / max_torque;

  auto apps_map_x = apps_map_x_from_rpm(rpm_act, vp);

  // Apply the Torque% -> APPS map
  return interp1d(vp.apps_map_y(), apps_map_x, torque_perc);
}


static inline AppsBrakePair apps_brake_from_accel(double target_acc,
                                                  double speed, int gear,
                                                  int rpm,
                                                  const VehicleParameters &vp) {
  constexpr double PI = std::numbers::pi;
  constexpr double G = 9.81;
  constexpr double AIR_DENSITY = 1.225;

  // Drag (X) and downforce (Z)
  double X = 0.5 * AIR_DENSITY * vp.scx() * std::pow(speed, 2);
  double Z = 0.5 * AIR_DENSITY * vp.scz() * std::pow(speed, 2);

  // Required torque at the rear wheels
  double coppia_ruote =
      (X * vp.wheel_radius_m()) + (((vp.mass_kg() * G) + Z) * vp.wheel_roll_coeff()) +
      ((vp.mass_kg() + ((4.0 * vp.wheel_inertia()) / std::pow(vp.wheel_radius_m(), 2)))) *
          vp.wheel_radius_m() * target_acc;

  if (coppia_ruote >= 0.0) {
    if (gear == 0)
      return { .brake_torque = 0.0, .apps = 0.0 };

    // Torque required at the engine
    double coppia_motore =
        (double)(coppia_ruote / vp.ratio_diff() / vp.gear_ratios().at(gear));

    double APS = apps_from_engine_torque(coppia_motore, rpm, vp);

    if (std::isnan(APS) || std::isinf(APS)) {
      APS = 0.0f;
    }

    return { .brake_torque = 0.0, .apps = APS };
  } else {
    // Compute the torque that the brake motor has to apply 
    double T_mot_freno_perm_mNm =
        ((((((-target_acc * vp.mass_kg() / 3) * (vp.wheel_radius_m() * 1000.0) / vp.brake_disc_radius_mm() / 4 /
             vp.brake_mu() / (PI / 4 * pow(vp.brake_piston_diameter_mm(), 2)) * 10) *
            vp.brake_pedal_down_distance_mm() / vp.brake_pedal_up_distance_mm() * 2 * (PI / 4 * pow(vp.brake_tilton_diameter_mm(), 2)) / 10) *
           vp.brake_pulley_diameter_mm() / 2000) /
          vp.brake_reducer() * 1000) /
         vp.brake_reducer_efficiency());

    if (T_mot_freno_perm_mNm <= vp.brake_min_torque())
      return { .brake_torque = 0.0, .apps = 0.0 };

    return { .brake_torque = T_mot_freno_perm_mNm / 1000.0, .apps = 0.0};
  }
}

static inline double normalizeAngle(double angle){
  while(angle > M_PI) angle -= (2 * M_PI);
  while(angle < -M_PI) angle += (2 * M_PI);
  return angle;
}

void OldLongitudinal::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) {
  m_vp = &vp;

  m_logger = logger;

  m_speed_lookforward_gain = p.get<double>("speed_lookforward_gain");
  m_minSpeedDistance = p.get<double>("minSpeedDistance");
  m_minSpeed = p.get<double>("minSpeed");
  m_max_accel_sq = p.get<double>("max_accel");
  m_max_accel_sq *= m_max_accel_sq;
  m_keep_launch = p.get<bool>("keep_launch");

  auto gear_p = p.subparams("gear_strategy");
  m_fixed_gear = gear_p.get_maybe<int>("fixed_gear");
  m_second_gear_from_lap = gear_p.get_maybe<int>("second_gear_from_lap");
  m_automatic_shifting_from_lap =  gear_p.get_maybe<int>("automatic_shifting_from_lap");
  
  int gear_strategy_count = 
      (m_fixed_gear.has_value()? 1:0)
    + (m_second_gear_from_lap.has_value()? 1:0)
    + (m_automatic_shifting_from_lap.has_value()? 1:0);

  if (gear_strategy_count != 1) {
    RCLCPP_FATAL(logger, "One and only one of the gear strategies must be selected!");
    throw std::invalid_argument("gear_strategy");
  }

  if (m_automatic_shifting_from_lap.has_value()) {
    m_min_up = gear_p.get<double>("min_up");
    m_max_up = gear_p.get<double>("max_up");
    m_min_down = gear_p.get<double>("min_down");
    m_max_down = gear_p.get<double>("max_down");
  }

  if (p.get<bool>("low_level_longitudinal_controller.simplified")) {
    m_simple_long_params = {
      .apps_p = p.get<double>("low_level_longitudinal_controller.apps_p"),
      .brake_p = p.get<double>("low_level_longitudinal_controller.brake_p")
    };
  } else {
    if (not p.get<bool>("low_level_longitudinal_controller.use_old_acceleration")) {
      m_new_accel_params = {
        .acceleration_p = p.get<double>("low_level_longitudinal_controller.acceleration_p")
      };
    }
  }

  auto dyn_speed_p = p.subparams("dynamicTargetSpeed");
  if (dyn_speed_p.get<bool>("enabled")) {
    m_dynamic_target_speed = {
      .slowLaps = dyn_speed_p.get<int>("slowLaps"),
      .maxSpeed = dyn_speed_p.get<double>("maxSpeed"),
      .targetSpeedWeight = dyn_speed_p.get<double>("targetSpeedWeight")
    };
  }
}


int OldLongitudinal::gear_target(
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

template <typename T>
static inline int sign(T x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

LongitudinalControl OldLongitudinal::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath& reference_path,
  const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
  int lap,
  std::optional<LateralControl> lat_ctrl
) {
  // Compute the speed lookforward.
  double speed_lookforward = m_minSpeedDistance;
  if (state.speed().has_value()) {
    speed_lookforward += m_speed_lookforward_gain * state.speed().value();
  }
  
  // Do we know where we are on the track?
  bool is_projection_valid = vehicle_path_projection.has_value();

  // Compute the maximum speed
  double maximum_speed = m_minSpeed;
  if (m_dynamic_target_speed.has_value() && lap > m_dynamic_target_speed->slowLaps) {
    // Use dynamic target speed

    if (!m_using_dynamic_speed) {
      RCLCPP_INFO(*this->m_logger, "Transitioning to dynamic target speed!");
      m_using_dynamic_speed = true;
    }

    // Get the target speed at the lookforward point
    if (is_projection_valid) {
      auto speed_target_ref = reference_path.advance_point(*vehicle_path_projection, speed_lookforward);

      if (auto max_speed_opt = reference_path.get_target_speed(speed_target_ref))
        maximum_speed = *max_speed_opt * m_dynamic_target_speed->targetSpeedWeight;
    }

    maximum_speed = std::clamp<double>(maximum_speed, m_minSpeed, m_dynamic_target_speed->maxSpeed);
  }

  if(!lat_ctrl.has_value()) // the lateral controller was not able to find a valid steering value
  {
    // stop the car
    maximum_speed = 0.0;
  }

  LongitudinalControl u(0.0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);

  // Longitudinal control
  int accel_sign = 0;
  double target_acceleration = NAN;
  if (state.speed().has_value()) {
    if (m_simple_long_params.has_value()) {
      // Use a simple P control (useful for the simulator)
      double error = maximum_speed - state.speed().value();
      accel_sign = sign(error);
      u.throttle = m_simple_long_params->apps_p * std::max(error, 0.0);
      u.brake = m_simple_long_params->brake_p * std::max(-error, 0.0);
    } else {
      // Compute the target acceleration
      if (m_new_accel_params.has_value())
        target_acceleration = (maximum_speed - *state.speed()) * m_new_accel_params->acceleration_p;
      else
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

  // Compute target gear
  if (m_second_gear_from_lap.has_value()) {
    if (lap >= m_second_gear_from_lap.value())
      u.gear = 2;
  }
  else if (m_automatic_shifting_from_lap.has_value()) {
    if (lap >= m_automatic_shifting_from_lap.value())
      u.gear = this->gear_target(accel_sign, state);
  }
  else if (m_fixed_gear.has_value()) {
    u.gear = m_fixed_gear.value();
  } else {
    assert(false && "None of the gear strategies were selected");
  }

  if (m_keep_launch)
    u.launch = Control::LaunchControl::Set;

  return u;
}

}// namespace old_longitudinal
}// namespace longitudinal
}// namespace separate_long_lat
}// namespace control
}// namespace control_node