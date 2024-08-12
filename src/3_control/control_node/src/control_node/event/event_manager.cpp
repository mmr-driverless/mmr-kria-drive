#include <rclcpp/qos.hpp>
#include <control_node/control/control.hpp>
#include <control_node/event/event_manager.hpp>
#include <mmr_base/configuration.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>

namespace control_node {
namespace event {

EventManager::EventManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger, const actuation::ActuatorManager& actuators)
  : m_logger(logger),
    m_as_state_sub(node->create_subscription<std_msgs::msg::Int8>(p.get<std::string>("as_state.topic"), p.parse_qos("as_state.qos"), std::bind(&EventManager::as_state_cb, this, std::placeholders::_1))),
    m_race_status_sub(node->create_subscription<std_msgs::msg::Int8>(p.get<std::string>("race_status.topic"), p.parse_qos("race_status.qos"), std::bind(&EventManager::race_status_cb, this, std::placeholders::_1))),
    m_stop_pub(node->create_publisher<std_msgs::msg::Bool>(p.get<std::string>("stop.topic"), p.parse_qos("stop.qos"))),
    m_actuators(actuators),
    m_event_state(EventState::Idle),
    m_enabled(p.get<bool>("enabled")),
    m_use_lc(p.get<bool>("use_lc")),
    m_launch_throttle(p.get<double>("launch_throttle")),
    m_launch_brake(p.get<double>("launch_brake")),
    m_launch_rpm(p.get<int>("launch_rpm")),
    m_lap_to_stop(p.get<int>("lap_to_stop")),
    m_launch_speed(p.get<double>("launch_speed")),
    m_standstill_speed(p.get<double>("standstill_speed")),
    m_stop_brake(p.get<double>("stop_brake")),
    m_lc_timeout(std::chrono::milliseconds(p.get<int>("launch_control_timeout_ms"))),
    m_self_is_disabled_but_requested_actuators_enable(false)
{

}

control::Control EventManager::run_fsm(std::chrono::milliseconds t, const estimation::IVehicleState& x, const control::Control& u) {
  switch (m_event_state) {
    case EventState::Idle:
      RCLCPP_INFO(m_logger, "Waiting for the AS State to be DRIVING.");
      m_event_state = EventState::WaitingForDriving;
      // We don't expect any actuator to actuate this...
      return control::Control(0.0, 0.0, m_launch_brake, control::Control::Clutch::Disengaged, 0, control::Control::LaunchControl::Unset);

    case EventState::WaitingForDriving:
      // Wait for AS driving
      if (m_as_state.has_value() && m_as_state.value() == AS::STATE::DRIVING) {
        RCLCPP_INFO(m_logger, "Waiting for the clutch to be disengaged and the gear to be 1.");
        m_event_state = EventState::WaitingForBaseState;
      }
      // We don't expect any actuator to actuate this...
      return control::Control(0.0, 0.0, m_launch_brake, control::Control::Clutch::Disengaged, 0, control::Control::LaunchControl::Unset);

    case EventState::WaitingForBaseState:
      // Wait for disengaged clutch and 1st gear
      if (x.clutch_is_engaged().has_value() && !x.clutch_is_engaged().value()) {
        if (x.gear().has_value() && x.gear().value() == 1) {
          RCLCPP_INFO(m_logger, "Enabling all actuators.");
          m_actuators.request_enable_all();
          m_event_state = EventState::WaitingForActuators;
        }
      }
      // We don't expect any actuator to actuate this...
      return control::Control(0.0, 0.0, m_launch_brake, control::Control::Clutch::Disengaged, 0, control::Control::LaunchControl::Unset);

    case EventState::WaitingForActuators:
      if (m_actuators.all_enabled()) {
        RCLCPP_INFO(m_logger, "Preparing the launch control.");
        m_wait_lc_start_time = t;
        m_event_state = EventState::Launch_SetLaunchControl;
      }
      // Actuators may start actuating this input at any time. We mantain the base state that we previously ensured the car was in.
      return control::Control(
        0.0,
        0.0,
        m_launch_brake,
        control::Control::Clutch::Disengaged,
        1,
        control::Control::LaunchControl::Unset
      );
    
    case EventState::Launch_SetLaunchControl:
      if (m_use_lc) {
        if (x.lc_is_active().has_value() && x.lc_is_active().value()) {
          RCLCPP_INFO(m_logger, "Succesfully activated the Launch Control");
          m_event_state = EventState::Launch_Rev;
        } else if (t - m_wait_lc_start_time > m_lc_timeout) {
          RCLCPP_WARN(m_logger, "The Launch Control check has timed out!");
          m_event_state = EventState::Launch_Rev;
        }
      } else {
        RCLCPP_WARN(m_logger, "Launch Control is disabled from config!");
        m_event_state = EventState::Launch_Rev;
      }

      if (m_event_state == EventState::Launch_Rev)
        RCLCPP_INFO(m_logger, "Revving the engine to %d RPM.", m_launch_rpm);

      return control::Control(
        u.steer,
        0.0,
        m_launch_brake,
        control::Control::Clutch::Disengaged,
        1,
        m_use_lc? control::Control::LaunchControl::Set : control::Control::LaunchControl::Unset
      );

    case EventState::Launch_Rev:
      if (x.rpm().has_value() && x.rpm() >= m_launch_rpm) {
        RCLCPP_INFO(m_logger, "Engaging clutch. Waiting for it to be engaged (or the speed to be larger than '%.1lf' m/s)", m_launch_speed);
        m_event_state = EventState::Launch_EngageClutch;
      }
      return control::Control(
        u.steer,
        m_launch_throttle,
        0.0,
        control::Control::Clutch::Disengaged,
        1,
        m_use_lc? control::Control::LaunchControl::Set : control::Control::LaunchControl::Unset
      );

    case EventState::Launch_EngageClutch:
      if ((x.speed().has_value() && x.speed().value() > m_launch_speed) || (x.clutch_is_engaged().has_value() && x.clutch_is_engaged().value())) {
        RCLCPP_INFO(m_logger, "PORCODDIO LA MACCHINA E' AUTONOMA!!");
        m_event_state = EventState::Driving;
      }
      return control::Control(
        u.steer,
        m_launch_throttle,
        0.0,
        control::Control::Clutch::Engaged,
        1,
        m_use_lc? control::Control::LaunchControl::Set : control::Control::LaunchControl::Unset
      );
    
    case EventState::Driving:
      if (m_lap.has_value() && m_lap.value() >= m_lap_to_stop) {
        RCLCPP_INFO(m_logger, "Target lap (%d) reached. Disengaging clutch.", m_lap_to_stop);
        m_event_state = EventState::Stop_DisengageClutch;
      }
      return u; // woah

    case EventState::Stop_DisengageClutch:
      if (x.clutch_is_engaged().has_value() && !x.clutch_is_engaged()) {
        RCLCPP_INFO(m_logger, "Clutch disengaged. Stopping the car.");
        m_event_state = EventState::Stop_Halt;
      }

      return control::Control(
        u.steer,
        0.0,
        0.0,
        control::Control::Clutch::Disengaged,
        0,
        control::Control::LaunchControl::Unset
      );

    case EventState::Stop_Halt:
      if (x.speed().has_value() && x.speed().value() <= m_standstill_speed) {
        RCLCPP_INFO(m_logger, "The car speed is below the standstill threshold (%.2lf m/s). Waiting %ld milliseconds while braking just to be sure...", m_standstill_speed, m_standstill_time.count());
        m_event_state = EventState::Stop_EnsureStandstill;
        m_standstill_start_time = t;
      }

      return control::Control(
        u.steer,
        0.0,
        m_stop_brake,
        control::Control::Clutch::Disengaged,
        0,
        control::Control::LaunchControl::Unset
      );

    case EventState::Stop_EnsureStandstill:
      if (t - m_standstill_start_time >= m_standstill_time) {
        RCLCPP_INFO(m_logger, "Aaand we're done!");
        m_event_state = EventState::FinishedOrEmergency;

        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        m_stop_pub->publish(msg);

        m_actuators.request_disable_all();
      }

      return control::Control(
        0.0,
        0.0,
        m_stop_brake,
        control::Control::Clutch::Disengaged,
        0,
        control::Control::LaunchControl::Unset
      );

    case EventState::FinishedOrEmergency:
      // Congratulations :) ... or maybe not :(

      // We don't expect any actuator to actuate this...
      return control::Control(0.0, 0.0, 0.0, control::Control::Clutch::Engaged, 0, control::Control::LaunchControl::Unset);

    default:
      assert(false && "Entered an invalid EventState.");
  }
}

control::Control EventManager::tick(std::chrono::nanoseconds t, const estimation::IVehicleState& x, const control::Control& u) {
  // If the EventManager is not active
  if (!m_enabled) {
    // Then we should enable all actuators (once)
    if (!m_self_is_disabled_but_requested_actuators_enable) {
      m_self_is_disabled_but_requested_actuators_enable = true;
      m_actuators.request_enable_all();
    }
    return u;
  }

  // If we just entered AS Emergency
  if (m_as_state == AS::STATE::EMERGENCY && m_event_state != EventState::FinishedOrEmergency) {
    // Disable all actuators and enter the FinishedOrEmergency (final) state
    m_actuators.request_disable_all();
    m_event_state = EventState::FinishedOrEmergency;
  }

  auto old_state = EventState::Invalid;
  
  control::Control ans = u;

  // Run the FSM until it doesn't change state.
  int i = 0;
  while (m_event_state != old_state) {
    old_state = m_event_state;
    
    if (i > (int)EventState::Invalid) {
      RCLCPP_ERROR(m_logger, "The FSM has performed an absurd amount of state transitions!");
      return ans;
    }

    ans = run_fsm(std::chrono::duration_cast<std::chrono::milliseconds>(t), x, u);
    ++i;
  }

  return ans;
}

}; // namespace event
}; // namespace control_node