#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_LQR_LQR_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_LQR_LQR_HPP

#include "control_node/control/separate_long_lat/lateral_control.hpp"
#include <control_node/control/separate_long_lat/ilateral_controller.hpp>

namespace control_node{
namespace control {
namespace separate_long_lat{
namespace lqr{

class LQR : public ILateralController{

  const VehicleParameters* m_vp;

  std::vector<std::string> m_raw_vectors_k;
  std::vector<std::pair<double, std::vector<double>>> m_k_pair;

  Eigen::Vector4f find_optimal_control_vector(double speed_in_module);

  public:

  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager& viz_mgr, rclcpp::Logger logger) override;

  virtual LateralControl control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
    int lap
  ) override;

};

}// namespace lqr
}// namespace separate_long_lat
}// namespace control
}// namespace control_node

#endif