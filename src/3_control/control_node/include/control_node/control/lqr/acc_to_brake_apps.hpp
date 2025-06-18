#include <array>
#include <cmath>
#include <vector>

#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace control {
namespace lqr { // literal theft from pure_pursuit_2023 folder

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

}; // namespace lqr
}; // namespace control
}; // namespace control_node