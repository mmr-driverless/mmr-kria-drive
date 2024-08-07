#ifndef CONTROLNODE_CONTROL_ICONTROLLER_HPP
#define CONTROLNODE_CONTROL_ICONTROLLER_HPP

#include <control_node/viz/viz_manager.hpp>

#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/control/control.hpp>
#include <control_node/path/reference_path.hpp>

#include <control_node/parameters.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace control {

struct IController {
  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr) = 0;
  virtual Control control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
  ) = 0;
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_ICONTROLLER_HPP
