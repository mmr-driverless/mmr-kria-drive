#ifndef CONTROLNODE_EVENT_EVENTMANAGER_HPP
#define CONTROLNODE_EVENT_EVENTMANAGER_HPP

#include <mmr_base/configuration.hpp>
#include <mmr_base/msg/detail/res_status__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <control_node/parameters.hpp>
#include <control_node/control/control.hpp>
#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/actuation/actuator_manager.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mmr_base/msg/res_status.hpp>

namespace control_node {
namespace event {

class EventManager {
  rclcpp::Logger m_logger;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_as_state_sub;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_race_status_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_stop_pub;

  const actuation::ActuatorManager& m_actuators;

  std::optional<AS::STATE> m_as_state;
  std::optional<int> m_lap;

  enum class EventState {
    Idle = 0,
    WaitingForDriving,
    WaitingForBaseState,
    WaitingForActuators,
    Launch_SetLaunchControl,
    Launch_RevBeforeEngage,
    Launch_EngageClutch,
    Launch_RevAfterEngage,
    Driving,
    Stop_DisengageClutch,
    Stop_WaitForNeutral,
    Stop_Halt,
    Stop_EnsureStandstill,
    FinishedOrEmergency,
    Invalid
  } m_event_state;

  bool m_enabled;
  double m_launch_throttle;
  double m_launch_brake;
  int m_lap_to_stop;
  
  double m_standstill_speed;
  
  std::chrono::milliseconds m_standstill_time;

  std::chrono::milliseconds m_rev_duration_before_engage;
  std::chrono::milliseconds m_rev_duration_after_engage;

  std::chrono::milliseconds m_fsm_step_start_time;

  double m_stop_light_brake;
  double m_stop_hard_brake;
  
  bool m_self_is_disabled_but_requested_actuators_enable; // fuck me

  control::Control run_fsm(std::chrono::milliseconds t, const estimation::IVehicleState& x, const control::Control& u);

public:
  EventManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger, const actuation::ActuatorManager& actuators);
  
  void as_state_cb(std::shared_ptr<const std_msgs::msg::Int8> msg) { m_as_state = (AS::STATE)msg->data; }
  void race_status_cb(std::shared_ptr<const std_msgs::msg::Int8> msg) { m_lap = msg->data; }

  control::Control tick(std::chrono::nanoseconds t, const estimation::IVehicleState& x, const control::Control& u);
};

}; // namespace event
}; // namespace control_node

#endif // !CONTROLNODE_EVENT_EVENTMANAGER_HPP