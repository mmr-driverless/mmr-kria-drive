#ifndef CONTROLNODE_CONTROL_LQR_LQR_HPP
#define CONTROLNODE_CONTROL_LQR_LQR_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node{
namespace control {
namespace lqr{

class LQR : public IController{

    const VehicleParameters* m_vp;

    std::vector<std::string> m_raw_vectors_k;
    std::vector<std::pair<double, std::vector<double>>> m_k_pair;

    // Longitudinal control is copy-paste from PurePursuit2023
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
  Eigen::Vector4f find_optimal_control_vector(double speed_in_module);

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

};// namespace lqr
};// namespace control
};// namespace control_node

#endif // !CONTROLNODE_CONTROL_LQR_LQR_HPP
