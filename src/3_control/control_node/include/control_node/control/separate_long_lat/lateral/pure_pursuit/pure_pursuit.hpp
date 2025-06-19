#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_PUREPURSUIT_PUREPURSUIT_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_PUREPURSUIT_PUREPURSUIT_HPP

#include <control_node/control/separate_long_lat/ilateral_controller.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <mmr_base/msg/pure_pursuit_log.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {
namespace pure_pursuit_{

class PurePursuit2023 : public ILateralController {

  std::optional<rclcpp::Logger> m_logger;

  rclcpp::Publisher<mmr_base::msg::PurePursuitLog>::SharedPtr m_log_pub;

  const VehicleParameters* m_vp;
  double m_minLookForward;
  double m_minLookForwardGain;
  double m_steerGain;

  typedef struct {
    double x;
    double y;
  } mmr_point_double;

  viz::VizManager* m_viz_mgr;
  float m_viz_lookforward_alpha;
  int m_viz_lookforward;

  void viz(std::optional<Eigen::Vector2d> target);

  static inline double lerp2(const double x, mmr_point_double start, mmr_point_double end) {
    const double M = end.y - start.y;
    const double X = (x - start.x) / (end.x - start.x);
    const double Q = start.y;

    return M * X + Q;
  }

  static inline double lerp3(const double x, mmr_point_double start, mmr_point_double p1, mmr_point_double end) {
    return x < p1.x? lerp2(x, start, p1) : lerp2(x, p1, end);
  }

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

}// namespace pure_pursuit
}// namespace separate_long_lat
}// namespace control
}// namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_PUREPURSUIT_PUREPURSUIT_HPP