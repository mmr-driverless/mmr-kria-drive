#ifndef CONTROLNODE_CONTROL_LONGSTEPRESPONSE_HPP
#define CONTROLNODE_CONTROL_LONGSTEPRESPONSE_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>

namespace control_node {
namespace control {
namespace long_step_response {

class LongStepResponse : public IController {
  const VehicleParameters* m_vp;
  
  double m_max_speed;
  double m_step_size;
  std::chrono::milliseconds m_zero_duration;
  std::chrono::milliseconds m_start_t;

  std::optional<rclcpp::Logger> m_logger;

  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_as_state_pub;
  
  enum class State {
    WaitingForGo,
    Waiting,
    Running,
    Finished
  } m_state;

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

}; // namespace long_step_response
}; //namespace control
}; //namespace control_node

#endif // !CONTROLNODE_CONTROL_LONGSTEPRESPONSE_HPP