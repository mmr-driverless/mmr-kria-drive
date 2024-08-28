#ifndef CONTROLNODE_VEHICLEPARAMETERS_HPP
#define CONTROLNODE_VEHICLEPARAMETERS_HPP

#include <control_node/parameters.hpp>
#include <rclcpp/logger.hpp>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

namespace control_node {

class VehicleParameters {
  double m_wheelbase_m; // Distance between front and rear axles [m]
  double m_lr_m; // Distance between CoM and rear axle [m]
  double m_steering_ratio; // Ratio between steering wheel and wheel angle (bicycle model) [1]
  double m_scx;
  double m_scz;
  double m_wheel_radius_m; // Wheel radius
  double m_wheel_roll_coeff; // Rolling resistance coefficient
  double m_wheel_inertia; // Inertia of a single wheel
  double m_differential_ratio; // Differential gear ratio
  double m_mass_kg; // Vehicle mass [kg]
  double m_brake_disc_radius_mm; // Brake disk radius [mm]
  double m_brake_mu; // Brake pad-disk friction coefficient
  double m_brake_piston_diameter_mm; // Piston diameter [mm]
  double m_brake_pedal_up_distance_mm; // Pedal pivot distance - ebs [mm]
  double m_brake_pedal_down_distance_mm; // Pedal pivot distance - tilton [mm]
  double m_brake_tilton_diameter_mm; // Tilton diameter [mm]
  double m_brake_pulley_diameter_mm; // Brake pulley diameter [mm]
  double m_brake_reducer; // Brake motor reducer
  double m_brake_reducer_efficiency; // Reducer efficiency
  double m_brake_min_torque; // Minimum brake motor torque
  std::vector<double> m_gear_ratios; 
  std::vector<double> m_NMOTVET_rpm;
  std::vector<double> m_CDC_Nm;
  std::vector<double> m_apps_map_rpm;
  Eigen::MatrixXd m_apps_map_x;
  std::vector<double> m_apps_map_y;

  rclcpp::Logger m_logger;

public:
  VehicleParameters(const Parameters& p, rclcpp::Logger logger)
    : m_wheelbase_m(p.get<double>("wheelbase_m")),
      m_lr_m(p.get<double>("lr_m")),
      m_steering_ratio(p.get<double>("steering_ratio")),
      m_scx(p.get<double>("scx")),
      m_scz(p.get<double>("scz")),
      m_wheel_radius_m(p.get<double>("wheel_radius_m")),
      m_wheel_roll_coeff(p.get<double>("wheel_roll_coeff")),
      m_wheel_inertia(p.get<double>("wheel_inertia")),
      m_differential_ratio(p.get<double>("differential_ratio")),
      m_mass_kg(p.get<double>("mass_kg")),
      m_brake_disc_radius_mm(p.get<double>("brake_disc_radius_mm")),
      m_brake_mu(p.get<double>("brake_mu")),
      m_brake_piston_diameter_mm(p.get<double>("brake_piston_diameter_mm")),
      m_brake_pedal_up_distance_mm(p.get<double>("brake_pedal_up_distance_mm")),
      m_brake_pedal_down_distance_mm(p.get<double>("brake_pedal_down_distance_mm")),
      m_brake_tilton_diameter_mm(p.get<double>("brake_tilton_diameter_mm")),
      m_brake_pulley_diameter_mm(p.get<double>("brake_pulley_diameter_mm")),
      m_brake_reducer(p.get<double>("brake_reducer")),
      m_brake_reducer_efficiency(p.get<double>("brake_reducer_efficiency")),
      m_brake_min_torque(p.get<double>("brake_min_torque")),
      m_gear_ratios(p.get<std::vector<double>>("gear_ratios")),
      m_NMOTVET_rpm(p.get<std::vector<double>>("nmotvet_rpm")),
      m_CDC_Nm(p.get<std::vector<double>>("cdc_Nm")),
      m_apps_map_rpm(p.get<std::vector<double>>("apps_map_rpm")),
      m_apps_map_y(p.get<std::vector<double>>("apps_map_y")),
      m_logger(logger)
  {
    auto map_x = p.get<std::vector<double>>("apps_map_x");
    auto M = m_apps_map_rpm.size();
    auto N = m_apps_map_y.size();

    if (map_x.size() != M * N) {
      throw std::invalid_argument("Wrong size for apps_map_x");
    }

    m_apps_map_x = Eigen::Map<Eigen::MatrixXd>(map_x.data(), N, M);
  }

  double wheelbase_m() const { return m_wheelbase_m; }
  double lr_m() const { return m_lr_m; }
  double steering_ratio() const { return m_steering_ratio; }
  double scx() const { return m_scx; }
  double scz() const { return m_scz; }
  double wheel_radius_m() const { return m_wheel_radius_m; }
  double wheel_roll_coeff() const { return m_wheel_roll_coeff; }
  double wheel_inertia() const { return m_wheel_inertia; }
  double ratio_diff() const { return m_differential_ratio; }
  double mass_kg() const { return m_mass_kg; }
  double brake_disc_radius_mm() const { return m_brake_disc_radius_mm; }
  double brake_mu() const { return m_brake_mu; }
  double brake_piston_diameter_mm() const { return m_brake_piston_diameter_mm; }
  double brake_pedal_up_distance_mm() const { return m_brake_pedal_up_distance_mm; }
  double brake_pedal_down_distance_mm() const { return m_brake_pedal_down_distance_mm; }
  double brake_tilton_diameter_mm() const { return m_brake_tilton_diameter_mm; }
  double brake_pulley_diameter_mm() const { return m_brake_pulley_diameter_mm; }
  double brake_reducer() const { return m_brake_reducer; }
  double brake_reducer_efficiency() const { return m_brake_reducer_efficiency; }
  double brake_min_torque() const { return m_brake_min_torque; }
  const std::vector<double>& gear_ratios() const { return m_gear_ratios; }
  const std::vector<double>& NMOVET_rpm() const { return m_NMOTVET_rpm; }
  const std::vector<double>& CDC_Nm() const { return m_CDC_Nm; }
  const Eigen::MatrixXd& apps_map_x() const { return m_apps_map_x; }
  const std::vector<double>& apps_map_y() const { return m_apps_map_y; }
  const std::vector<double>& apps_map_rpm() const { return m_apps_map_rpm; }
};

};

#endif // !CONTROLNODE_VEHICLEPARAMETERS_HPP
