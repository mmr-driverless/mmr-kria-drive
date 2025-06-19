#ifndef CONTROLNODE_CONTROL_CONTROL_HPP
#define CONTROLNODE_CONTROL_CONTROL_HPP

namespace control_node {
namespace control {

struct Control {
  enum class LaunchControl {
    Set,
    Unset
  };
  enum class Clutch {
    Engaged,
    Disengaged
  };

  Control(double steer, double throttle, double brake, Clutch clutch, int gear, LaunchControl launch)
    : steer(steer), throttle(throttle), brake(brake), clutch(clutch), gear(gear), launch(launch) {}
  
  // Steering wheel angle [deg]
  double steer;

  // APPS [%]
  double throttle;

  // Brake motor torque [Nm]
  double brake;

  // Clutch state
  Clutch clutch;

  // Gear
  int gear;

  // Launch control
  LaunchControl launch;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROL_HPP