#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_SEPARATELONGLAT_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_SEPARATELONGLAT_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/control/separate_long_lat/ilongitudinal_controller.hpp>
#include <control_node/control/separate_long_lat/ilateral_controller.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

class SeparateLongitudinalLateralController : public IController {
  std::vector<std::pair<int, std::unique_ptr<ILongitudinalController>>> m_longitudinal_controllers;
  std::vector<std::pair<int, std::unique_ptr<ILateralController>>> m_lateral_controllers;

  int m_switch_lap;
  std::pair<int, int> m_longitudinal_controller_idx;
  std::pair<int, int> m_lateral_controller_idx;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) override;

  virtual Control control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
    int lap
  ) override;
};

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_SEPARATELONGLAT_HPP