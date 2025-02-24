#include <chrono>
#include <control_node/control/long_step_response/long_step_response.hpp>
#include <control_node/control/pure_pursuit_2023/acc_to_brake_apps.hpp>
#include <mmr_base/configuration.hpp>

namespace control_node {
namespace control {
namespace long_step_response {


void LongStepResponse::init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager&, rclcpp::Logger logger) {
  m_vp = &vp;
  m_as_state_pub = node.create_publisher<std_msgs::msg::Int8>(p.get<std::string>("as_state.topic"), p.parse_qos("as_state.qos"));
  m_zero_duration = std::chrono::milliseconds(p.get<int>("zero_duration_ms"));
  m_step_size = p.get<double>("step_size");
  m_max_speed = p.get<double>("max_speed");
  m_state = State::WaitingForGo;
  m_logger = logger;
}

Control LongStepResponse::control(
  std::chrono::nanoseconds t,
  const estimation::IVehicleState& state,
  const path::ReferencePath&,
  const std::optional<path::ReferencePath::PointRef>&,
  int
) {
  auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t);

  double throttle = 0.0;

  if (m_state == State::WaitingForGo) {
    if (state.res_go().has_value() && state.res_go().value()) {
      RCLCPP_INFO(*m_logger, "Go received!");
      std_msgs::msg::Int8 msg;
      msg.data = AS::STATE::DRIVING;
      m_as_state_pub->publish(msg);
      m_state = State::Waiting;
    }
  }

  if (m_state == State::Waiting) {
    if (state.res_bag().has_value() && state.res_bag().value()) {
      m_state = State::Running;
      m_start_t = t_ms;
      RCLCPP_INFO(*m_logger, "Bag button received - running!");
    }
  }

  if (m_state == State::Running) {
    bool over_speed = !state.speed().has_value() || state.speed().value() >= m_max_speed;
    bool go_off = !state.res_go().has_value() || !state.res_go().value();

    if (over_speed || go_off) {
      RCLCPP_INFO(*m_logger, "Stop (over_speed=%s), (go_off=%s)", over_speed?"T":"F", go_off?"T":"F");
      m_state = State::Finished;
    }
    else if (state.speed().has_value() && state.gear().has_value() && state.rpm().has_value()) {
      double target_acc = 0.0;

      if (t_ms - m_start_t >= m_zero_duration)
        target_acc = m_step_size;

      throttle = pure_pursuit_2023::apps_brake_from_accel(target_acc, state.speed().value(), state.gear().value(), state.rpm().value(), *m_vp).apps;
    }
  }

  return Control(0.0, throttle, 0.0, Control::Clutch::Engaged, 0, Control::LaunchControl::Unset);
}

}; // namespace long_step_response
}; // namespace control
}; // namespace control_node