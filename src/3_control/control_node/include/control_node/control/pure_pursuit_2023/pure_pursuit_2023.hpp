#ifndef CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP
#define CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>

#include <mmr_base/msg/pure_pursuit_log.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

class PurePursuit2023 : public IController {
  std::optional<rclcpp::Logger> m_logger;

  rclcpp::Publisher<mmr_base::msg::PurePursuitLog>::SharedPtr m_log_pub;

  const VehicleParameters* m_vp;
  double m_minLookForward;
  double m_minLookForwardGain;
  double m_steerGain;

  double m_minSpeedDistance;
  double m_speed_lookforward_gain;
  
  double m_minSpeed;

  double m_max_accel_sq;

  std::optional<bool>m_use_simulator_steering;

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

  viz::VizManager* m_viz_mgr;
  float m_viz_lookforward_alpha;
  int m_viz_lookforward;

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

  virtual Control control(
    std::chrono::nanoseconds t,
    const estimation::IVehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
    int lap
  ) override;
};

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP