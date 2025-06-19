#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINAL_OLDLONGITUDINAL_OLDLONGITUDINAL_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINAL_OLDLONGITUDINAL_OLDLONGITUDINAL_HPP

#include "control_node/control/separate_long_lat/ilongitudinal_controller.hpp"
#include <control_node/vehicle_parameters.hpp>
#include <mmr_base/msg/pure_pursuit_log.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {
namespace longitudinal {
namespace old_longitudinal {

class OldLongitudinal : ILongitudinalController
{
  std::optional<rclcpp::Logger> m_logger;

  const VehicleParameters* m_vp;
  double m_minLookForward;
  double m_minLookForwardGain;
  double m_steerGain;

  double m_minSpeedDistance;
  double m_speed_lookforward_gain;
  
  double m_minSpeed;

  double m_max_accel_sq;

  struct SimplifiedLongitudinalControlParams {
    double apps_p;
    double brake_p;
  };

  struct NewAccelerationParams {
    double acceleration_p;
  };

  struct DynamicTargetSpeedParams {
    int slowLaps;
    double maxSpeed;
    double targetSpeedWeight;
  };

  bool m_keep_launch;

  std::optional<int> m_automatic_shifting_from_lap;
  std::optional<int> m_second_gear_from_lap;
  std::optional<int> m_fixed_gear;

  double m_min_up, m_max_up;
  double m_min_down, m_max_down;

  std::optional<NewAccelerationParams> m_new_accel_params;
  std::optional<SimplifiedLongitudinalControlParams> m_simple_long_params;
  std::optional<DynamicTargetSpeedParams> m_dynamic_target_speed;
  
  typedef struct {
    double x;
    double y;
  } mmr_point_double;

  bool m_using_dynamic_speed = false;


  void viz(std::optional<Eigen::Vector2d> target);
  int gear_target(int acceleration_sign, const estimation::IVehicleState& state);

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

  virtual LongitudinalControl control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
    int lap
  ) override;
};

}// namespace old_longitudinal
}// namespace longitudinal
}// namespace separate_long_lat
}// namespace control
}// namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINAL_OLDLONGITUDINAL_OLDLONGITUDINAL_HPP