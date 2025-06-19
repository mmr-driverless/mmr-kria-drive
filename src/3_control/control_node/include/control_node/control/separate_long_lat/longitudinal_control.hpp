#include <control_node/control/control.hpp>

#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROL_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROL_HPP

namespace control_node {
namespace control {
namespace separate_long_lat {

struct LongitudinalControl {
  LongitudinalControl(double throttle, double brake, Control::Clutch clutch, int gear, Control::LaunchControl launch)
    : throttle(throttle), brake(brake), clutch(clutch), gear(gear), launch(launch) {}

  // APPS [%]
  double throttle;

  // Brake motor torque [Nm]
  double brake;

  // Clutch state
  Control::Clutch clutch;

  // Gear
  int gear;

  // Launch control
  Control::LaunchControl launch;
};

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROL_HPP