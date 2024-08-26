#include <cmath>
#include <vector>

#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

struct AppsBrakePair {
  double brake_torque;
  double apps;
};

static inline AppsBrakePair apps_brake_from_accel(double target_acc,
                                                  double speed, int gear,
                                                  int rpm,
                                                  const VehicleParameters &vp) {
  constexpr double PI = std::numbers::pi;
  constexpr double G = 9.81;
  constexpr double AIR_DENSITY = 1.225;

  // Drag (X) and downforce (Z)
  double X = 0.5 * AIR_DENSITY * vp.cx() * vp.sx() * std::pow(speed, 2);
  double Z = 0.5 * AIR_DENSITY * vp.cz() * vp.sz() * std::pow(speed, 2) * vp.cz() * vp.sz();

  double M = 300.0; // vp.mass_kg()

  // Required torque at the rear wheels
  double coppia_ruote =
      (X * vp.wheel_radius_m()) + (((M * G) + Z) * vp.wheel_roll_coeff()) +
      ((M + ((4.0 * vp.wheel_inertia()) / std::pow(vp.wheel_radius_m(), 2)))) *
          vp.wheel_radius_m() * target_acc;

  if (coppia_ruote >= 0.0) {
    if (gear == 0)
      return { .brake_torque = 0.0, .apps = 0.0 };

    // Torque required at the engine
    double coppia_motore =
        (double)(coppia_ruote / vp.ratio_diff() / vp.gear_ratios().at(gear));

    /*
      Find the maximum torque of the engine at the current RPM
      by linear interpolation of the torque curve obtained at the dyno
    */
    int NMOTVET_SIZE = vp.NMOVET_rpm().size();
    int RPMBOUNDINF = NMOTVET_SIZE - 1;
    int RMPBOUNDSUP = RPMBOUNDINF;

    double CDCINTERPOLATO = 0.0, m = 0.0, q = 0.0;

    for (int i = 1; i < NMOTVET_SIZE; ++i) {
      if (rpm <= vp.NMOVET_rpm().at(i)) {
        RPMBOUNDINF = i - 1;
        RMPBOUNDSUP = i;
        break;
      }
    }

    m = (double)((double)(vp.CDC_Nm().at(RMPBOUNDSUP) - vp.CDC_Nm().at(RPMBOUNDINF)) /
                 (double)(vp.NMOVET_rpm().at(RMPBOUNDSUP) - vp.NMOVET_rpm().at(RPMBOUNDINF)));

    q = (double)((double)((double)(vp.NMOVET_rpm().at(RMPBOUNDSUP) * vp.CDC_Nm().at(RPMBOUNDINF)) -
                          (double)(vp.NMOVET_rpm().at(RPMBOUNDINF) * vp.CDC_Nm().at(RMPBOUNDSUP))) /
                 (double)(vp.NMOVET_rpm().at(RMPBOUNDSUP) - vp.NMOVET_rpm().at(RPMBOUNDINF)));

    CDCINTERPOLATO = (double)(m * static_cast<double>(rpm)) + q;

    // Assume that the torque produced by the motor is directly proportional to
    // APPS (where APPS=1 => max torque at the current RPM)
    double APS = (double)(coppia_motore / CDCINTERPOLATO);

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

    return { .brake_torque = T_mot_freno_perm_mNm / 1000.0, .apps = 0.0};
  }
}

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node