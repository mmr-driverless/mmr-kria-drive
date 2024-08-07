#ifndef CONTROLNODE_CONTROL_CONTROL_HPP
#define CONTROLNODE_CONTROL_CONTROL_HPP

namespace control_node {
namespace control {

class Control {
public:
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
  
  double steer;
  double throttle;
  double brake;
  Clutch clutch;
  int gear;
  LaunchControl launch;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROL_HPP