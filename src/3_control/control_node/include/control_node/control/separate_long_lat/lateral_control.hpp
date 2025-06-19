#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_LATERALCONTROL_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_LATERALCONTROL_HPP

namespace control_node {
namespace control {
namespace separate_long_lat {

struct LateralControl {
  LateralControl(double steer) : steer(steer) {}

  // Steering wheel angle [deg]
  double steer;
};

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROL_HPP