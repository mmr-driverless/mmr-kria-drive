#ifndef CONTROLNODE_CONTROL_ICONTROLLER_HPP
#define CONTROLNODE_CONTROL_ICONTROLLER_HPP

#include <control_node/estimation/vehicle_state.hpp>
#include <control_node/control/control.hpp>
#include <control_node/path/reference_path.hpp>

namespace control_node {
namespace control {

struct IController {
  virtual Control control(
    const estimation::VehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
  ) = 0;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_ICONTROLLER_HPP
