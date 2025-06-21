#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_ILATERALCONTROLLER_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_ILATERALCONTROLLER_HPP

#include <control_node/viz/viz_manager.hpp>

#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/path/reference_path.hpp>

#include <control_node/parameters.hpp>
#include <control_node/vehicle_parameters.hpp>

#include <control_node/control/separate_long_lat/lateral_control.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

struct ILateralController {
  virtual ~ILateralController() = default;
  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) = 0;
  virtual std::optional<LateralControl> control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
    int lap
  ) = 0;
};

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_ILATERALCONTROLLER_HPP
