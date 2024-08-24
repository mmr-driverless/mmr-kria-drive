#ifndef CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP
#define CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

class PurePursuit2023 : public IController {
  std::optional<rclcpp::Logger> m_logger;

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr m_targetSpeedPub;


  const VehicleParameters* m_vp;
  double m_minLookForward;
  double m_minLookForwardGain;
  double m_steerGain;

  double m_minSpeedDistance;
  double m_minSpeed;

  double m_min_throttle;
  bool m_simplified_longitudinal_control_enabled;
  double m_simple_long_apps_p;
  double m_simple_long_brake_p;
  double m_ll_accel_lookforward;
  double m_ll_accel_k_smooth;

  bool m_second_gear_on_second_lap;

  struct {
    bool enabled;
    int slowLaps;
    double k_smooth;
    double maxSpeed;
    double targetSpeedWeight;
  } m_dynamicTargetSpeed;
  

  bool m_using_dynamic_speed = false;
  double m_smoothedSpeed = 0;
  double m_smoothedAccel = 0;

  viz::VizManager* m_viz_mgr;
  float m_viz_lookforward_alpha;
  int m_viz_lookforward;

  void pub_target_speed(std::chrono::nanoseconds t, double speed);
  void viz(std::optional<Eigen::Vector2d> target);

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