#ifndef CONTROLNODE_EVENT_EVENTMANAGER_HPP
#define CONTROLNODE_EVENT_EVENTMANAGER_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mmr_base/msg/race_status.hpp>
#include <mmr_base/configuration.hpp>

#include <control_node/parameters.hpp>
#include <control_node/control/control.hpp>
#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/actuation/actuator_manager.hpp>
#include <control_node/path/reference_path.hpp>

namespace control_node {
namespace event {

class EventManager {
  rclcpp::Logger m_logger;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_as_state_sub;
  rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr m_race_status_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_stop_pub;

  const actuation::ActuatorManager& m_actuators;

  std::optional<AS::STATE> m_as_state;
  std::optional<int> m_lap;

  struct RequiredSignals {
    bool speed;
    bool position;
    bool yaw;
    bool trajectory;

    inline bool operator==(const RequiredSignals& other) const = default;
    bool all() const { return speed && position && yaw && trajectory; }
  };

  std::optional<RequiredSignals> m_required_signals;

  enum class EventState {
    Idle = 0,
    WaitingForDriving,
    WaitingForBaseState,
    WaitingForActuators,
    WaitingForSignals,
    Launch_SetLaunchControl,
    Launch_RevBeforeEngage,
    Launch_EngageClutch,
    Launch_RevAfterEngage,
    Driving_WithLC,
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
  std::optional<int> m_lap_to_stop;
  
  double m_standstill_speed;
  
  std::chrono::milliseconds m_standstill_time;

  std::chrono::milliseconds m_rev_duration_before_engage;
  std::chrono::milliseconds m_rev_duration_after_engage;

  std::chrono::milliseconds m_fsm_step_start_time;
  std::chrono::milliseconds m_lc_duration_after_launch;

  std::optional<std::chrono::milliseconds> m_mission_duration;

  double m_stop_light_brake;
  double m_stop_hard_brake;

  bool m_wait_for_required_signals;
  
  bool m_self_is_disabled_but_requested_actuators_enable; // fuck me

  control::Control run_fsm(std::chrono::milliseconds t, const estimation::IVehicleState& x, const control::Control& u, const path::ReferencePath& refpath);

public:
  EventManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger, const actuation::ActuatorManager& actuators);
  
  void as_state_cb(std::shared_ptr<const std_msgs::msg::Int8> msg) { m_as_state = (AS::STATE)msg->data; }
  void race_status_cb(std::shared_ptr<const mmr_base::msg::RaceStatus> msg) { m_lap = msg->current_lap; }

  control::Control tick(std::chrono::nanoseconds t, const estimation::IVehicleState& x, const control::Control& u, const path::ReferencePath& refpath);

  inline int lap() const { return m_lap.value_or(0); }
};

}; // namespace event
}; // namespace control_node

#endif // !CONTROLNODE_EVENT_EVENTMANAGER_HPP