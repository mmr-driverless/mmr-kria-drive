#ifndef CONTROLNODE_CONTROL_ICONTROLLER_HPP
#define CONTROLNODE_CONTROL_ICONTROLLER_HPP

#include <control_node/estimation/vehicle_state.hpp>
#include <control_node/control/control.hpp>
#include <control_node/path/reference_path.hpp>

namespace control_node {
namespace control {

struct IController {
  constexpr virtual bool is_path_follower() = 0;

  virtual Control control(const estimation::VehicleState& state) = 0;
  virtual Control control(
    const estimation::VehicleState& state,
    const path::ReferencePath& reference_path
  ) = 0;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_ICONTROLLER_HPP
