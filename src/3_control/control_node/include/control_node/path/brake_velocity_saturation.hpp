#ifndef CONTROLNODE_PATH_BRAKEVELOCITYSATURATION_HPP
#define CONTROLNODE_PATH_BRAKEVELOCITYSATURATION_HPP

#include <cmath>
#include <span>

namespace control_node {
namespace path {
namespace braking {

/*
  Returns the speed achieved with constant acceleration
  in the specified space starting at the specified velocity
*/
static inline double speed_from_accel_and_displacement(double speed0, double ds, double a) {
  return std::sqrt(speed0 * speed0 + 2 * a * ds);
}

static inline void saturate_velocity_with_brake_potential(const std::span<double>& waypoint_distances, std::span<double> speed_profile, double max_deceleration, bool is_closed) {
  /*
  A critical braking point is a point such that the speed profile is not guaranteed to have a feasible
  deceleration.

  Assuming the braking potential is a constant acceleration, we solve each of such critical braking points
  by saturating the entire velocity profile such that there is always enough braking potential to reach the
  critical braking point at the desired speed.
  */


  int N = speed_profile.size();
  
  // The first critical point is the last waypoint (we don't have a better guess).
  double crit_speed = speed_profile[N-1];
  double crit_dist = waypoint_distances[N-2];

  // Traverse from the end to the start to saturate the velocity w.r.t each critical point.
  for (int i = N-2; i >= 0; --i) {
    // Compute the maximum speed at this waypoint from the braking potential,
    // target speed and distance from critical braking point.
    double max_speed = speed_from_accel_and_displacement(crit_speed, crit_dist, max_deceleration);
    speed_profile[i] = std::min(speed_profile[i], max_speed);

    // Any point with speed < max_speed may be a critical braking point (actually, only local minimums, but whatever)
    if (speed_profile[i] < max_speed) {
      crit_speed = speed_profile[i];
      crit_dist = 0;
    }

    if (i > 0)
      crit_dist += waypoint_distances[i-1];
  }

  if (not is_closed)
    return;

  /* If the path is closed, the end and start sections should be continuous but right now they're not.
      Perform the same algorithm as before, but exit as soon as we find a new critical point (we've already processed them)
  */
  for (int i = N-1; i >= 1; --i) {
    double max_speed = speed_from_accel_and_displacement(crit_speed, crit_dist, max_deceleration);
    speed_profile[i] = std::min(speed_profile[i], max_speed);

    // If this is would seem to be a new critical point
    if (speed_profile[i] < max_speed) {
      // Then we're done, because we already solved them all
      return;
    }

    crit_dist += waypoint_distances[i-1];
  }
}

}; // namespace braking
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_BRAKEVELOCITYSATURATION_HPP