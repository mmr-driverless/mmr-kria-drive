#include <cmath>
#include <control_node/viz/viz_manager.hpp>
#include <control_node/parameters.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/control/lqr/lqr.hpp>
#include <control_node/control/lqr/acc_to_brake_apps.hpp>
#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace control_node{
namespace control{
namespace lqr{

void LQR::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger)
{
    m_vp = &vp;
    // m_log_pub = node.create_publisher<mmr_base::msg::PurePursuitLog>(p.get<std::string>("log.topic"), p.parse_qos("log.qos_override")); // TODO: think about how ot implement this
    
    // get lqr specfic subparams //
    m_raw_vectors_k = p.get<std::vector<std::string>>("control_vectors");
    for (const auto& vec_str : m_raw_vectors_k) {
        std::stringstream ss(vec_str);
        double first_value;
        std::vector<double> values;
        if (ss >> first_value) {
            double num;
            while (ss >> num) {
                values.push_back(num);
            }
            m_k_pair.emplace_back(first_value, values);
        }
    }
    // ! get lqr specfic subparams //
    
    // get powertrain management subparams //
    m_minLookForward = p.get<double>("minLookForward");
    m_minLookForwardGain = p.get<double>("minLookForwardGain");
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

    // ! get powertrain management subparams //

    // get speed management subparams //
    auto dyn_speed_p = p.subparams("dynamicTargetSpeed");
    if (dyn_speed_p.get<bool>("enabled")) {
      m_dynamic_target_speed = {
        .slowLaps = dyn_speed_p.get<int>("slowLaps"),
        .maxSpeed = dyn_speed_p.get<double>("maxSpeed"),
        .targetSpeedWeight = dyn_speed_p.get<double>("targetSpeedWeight")
      };
    }
    // ! get speed management subparams //

    // // get rviz subparams
    // m_viz_mgr = &viz_mgr;
    // auto viz_p = p.subparams("target_marker");
    // double scale = viz_p.get<double>("diameter");
    // m_viz_lookforward = viz_mgr.get_new(
    //   viz::msgs::Marker::CYLINDER,
    //   viz_p.parse_rgba("color", m_viz_lookforward_alpha),
    //   { scale, scale, 0.01 }
    // );
    // // ! get rviz subparams

    RCLCPP_INFO(logger, "LQR CONTROLLER INITIALIZED SUCCESSFULLY");
}

Eigen::Vector3f crossProduct(const Eigen::Vector3f& A, const Eigen::Vector3f& B) {
    return { 
        0, 
        0, 
        A[0] * B[1] - A[1] * B[0]
    };
}

double get_sign(double Ax, double Ay, double Bx, double By, double theta) 
{
    Eigen::Vector3f A(cos(theta), sin(theta), 0);
    
    Eigen::Vector3f B(Bx - Ax, By - Ay, 0);
    
    Eigen::Vector3f cross = crossProduct(A, B);
    const double epsilon = 1e-9;
    if (std::abs(cross[2]) < epsilon) {
        return 0.0; // Avoid division by zero
    }
    return cross[2]/std::abs(cross[2]);
}

double get_angular_deviation(double a, double b) 
{
    double diff = std::fmod(b - a, 2.0 * M_PI);
    if (diff > M_PI)
        diff -= 2.0 * M_PI;
    else if (diff < -M_PI)
        diff += 2.0 * M_PI;
    return diff;
}

template <typename T>
static inline int sign(T x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}


std::tuple<double, Eigen::Vector2d> get_lateral_deviation_components(const double angular_dev, const double closest_point_tangent, const double x_velocity, const double y_velocity)
{
    double Vx = x_velocity;
    double Vy = y_velocity;
    double lateral_deviation_speed = Vy*std::cos(angular_dev) + Vx*std::sin(angular_dev); // this is the speed of the car in the direction of the tangent line

    // Now reconstruct the perpendicular component into the original frame of reference in order to output it as a vector
    Eigen::Vector2d d_perp(-std::sin(closest_point_tangent), std::cos(closest_point_tangent));
    return {lateral_deviation_speed, lateral_deviation_speed * d_perp};
}

Eigen::Vector4f LQR::find_optimal_control_vector(double speed_in_module)
{
    Eigen::Vector4f optimal_control_vector;

    int closest_velocity_index = 0;
    double smallest_velocity_gap = 10e4;

    for (size_t i = 0; i < m_k_pair.size(); i++) 
        {
            // calculate the difference between speed_in_module and the velocity associated to the current control vector
            double velocity_gap = std::abs(speed_in_module - m_k_pair[i].first);
            if (velocity_gap < smallest_velocity_gap) 
            {
                closest_velocity_index = i;
                smallest_velocity_gap = velocity_gap;
            }
        }

    std::vector<double> v = m_k_pair[closest_velocity_index].second;
    optimal_control_vector << v[0], v[1], v[2], v[3];
    return optimal_control_vector;
}

double get_feedforward_term(const double K_3, const double mass, const double long_speed, const double radius, const double frontal_lenght, const double rear_lenght, const double C_alpha_rear, const double C_alpha_front)
{
    double df_c1 = (mass*std::pow(long_speed,2))/(radius*(rear_lenght+frontal_lenght));
    double df_c2 = (rear_lenght / (2*C_alpha_front))-(frontal_lenght / (2*C_alpha_rear)) + (frontal_lenght / (2*C_alpha_rear))*K_3;
    double df_c3 = (rear_lenght+frontal_lenght)/radius;
    double df_c4 = (rear_lenght/radius)*K_3;
    return df_c1*df_c2+df_c3-df_c4;
}

Control LQR::control(
std::chrono::nanoseconds t,
const estimation::IVehicleState& state,
const path::ReferencePath& reference_path,
const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
int lap
) {
  // Actual LQR logic
  // I get from the function arguments the state of the car, the path and the projection of the car on the path (I will never stop glazing shibodd for this)
  
  Control u(0.0, 0, 0.0, Control::Clutch::Engaged, 1, Control::LaunchControl::Unset);
  // calculate lateral deviation
  double lateral_deviation_module = 0.0;
  std::optional<Eigen::Vector2d> projection_point;
  bool is_projection_valid = vehicle_path_projection.has_value();
  if(is_projection_valid)
  {
      projection_point = reference_path.get_position(vehicle_path_projection.value());        
      if(projection_point.has_value())
      {
          lateral_deviation_module = std::hypot(projection_point->x() - state.position()->x(), projection_point->y() - state.position()->y());
      }    
  }
  double closest_point_curvature = reference_path.get_curvature(vehicle_path_projection.value()).value();
  double closest_point_curvature_radius = 1.0 / closest_point_curvature;
  double car_yaw = state.yaw().value();
  double closest_point_tangent = reference_path.get_track_yaw(vehicle_path_projection.value()).value();
  
  double lateral_deviation_sign = get_sign(projection_point->x(), projection_point->y(), state.position()->x(), state.position()->y(), closest_point_tangent);
  double lateral_deviation = lateral_deviation_module * lateral_deviation_sign;
  double angular_deviation = get_angular_deviation(closest_point_tangent, car_yaw);
  double velocity_x = state.speed().value();
  double velocity_y = 0.0; // it is not my fault lol. FG
  auto [lateral_deviation_speed, v_ld] = get_lateral_deviation_components(angular_deviation, closest_point_tangent, velocity_x, velocity_y);
  double angular_deviation_speed = state.yaw_rate().value();
  Eigen::Vector4f x;
  x << lateral_deviation, lateral_deviation_speed, angular_deviation, angular_deviation_speed;
  
  Eigen::Vector4f optimal_control_vector = find_optimal_control_vector(velocity_x);
  double K_3 = optimal_control_vector[2];
  double steering = -optimal_control_vector.dot(x); 
  double delta_f = get_feedforward_term(K_3, m_vp->mass_kg(), velocity_x, closest_point_curvature_radius, m_vp->lr_m(), m_vp->lr_m(), m_vp->C_alpha_front(), m_vp->C_alpha_rear());
  steering = steering + delta_f;
  u.steer = steering;
  // computation of longitudinal control (copy-paste from pure_pursuit_2023) //
  double speed_lookforward = m_minSpeedDistance;
  if (state.speed().has_value()){
      speed_lookforward += m_speed_lookforward_gain * state.speed().value();
  }
  // Compute the maximum speed
  double maximum_speed = m_minSpeed;
  if (m_dynamic_target_speed.has_value() && lap > m_dynamic_target_speed->slowLaps) {
    // Use dynamic target speed
  
    if (!m_using_dynamic_speed) {
      m_using_dynamic_speed = true;
    }
  
    // Get the target speed at the lookforward point
    if (is_projection_valid) {
      auto speed_target_ref = reference_path.advance_point(*vehicle_path_projection, speed_lookforward);
      
      if (auto max_speed_opt = reference_path.get_target_speed(speed_target_ref))
        maximum_speed = *max_speed_opt;
    }
  
    maximum_speed = std::clamp<double>(maximum_speed, m_minSpeed, m_dynamic_target_speed->maxSpeed);
  }
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

}; // namespace lqr
}; // namespace control
}; // namespace contol_node