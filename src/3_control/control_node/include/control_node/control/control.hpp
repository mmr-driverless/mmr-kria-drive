#ifndef CONTROLNODE_CONTROL_CONTROL_HPP
#define CONTROLNODE_CONTROL_CONTROL_HPP

namespace control_node {
namespace control {

class Control {
public:
  Control(double steer, double throttle, double brake, double clutch, int gear, bool launch)
    : steer(steer), throttle(throttle), brake(brake), clutch(clutch), gear(gear), launch(launch) {}

  double steer;
  double throttle;
  double brake;
  double clutch;
  int gear;
  bool launch;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROL_HPP