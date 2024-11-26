#include <control_node/control/control.hpp>
#include <control_node/event/event_manager.hpp>

#include <stdexcept>
#include <cassert>

namespace control_node {
namespace event {

EventManager::EventManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger, const actuation::ActuatorManager& actuators)
  : m_logger(logger),
    m_as_state_sub(node->create_subscription<std_msgs::msg::Int8>(p.get<std::string>("as_state.topic"), p.parse_qos("as_state.qos"), std::bind(&EventManager::as_state_cb, this, std::placeholders::_1))),
    m_race_status_sub(node->create_subscription<mmr_base::msg::RaceStatus>(p.get<std::string>("race_status.topic"), p.parse_qos("race_status.qos"), std::bind(&EventManager::race_status_cb, this, std::placeholders::_1))),
    m_stop_pub(node->create_publisher<std_msgs::msg::Bool>(p.get<std::string>("stop.topic"), p.parse_qos("stop.qos"))),
    m_actuators(actuators),
    m_event_state(EventState::Idle),
    m_enabled(p.get<bool>("enabled")),
    m_launch_throttle(p.get<double>("launch_throttle")),
    m_launch_brake(p.get<double>("launch_brake")),
    m_lap_to_stop(p.get_maybe<int>("lap_to_stop")),
    m_standstill_speed(p.get<double>("standstill_speed_m_s")),
    m_standstill_time(std::chrono::milliseconds(p.get<int>("standstill_time_ms"))),
    m_rev_duration_before_engage(std::chrono::milliseconds(p.get<int>("rev_duration_before_engage_ms"))),
    m_rev_duration_after_engage(std::chrono::milliseconds(p.get<int>("rev_duration_after_engage_ms"))),
    m_lc_duration_after_launch(std::chrono::milliseconds(p.get<int>("lc_duration_after_launch_ms"))),
    m_stop_light_brake(p.get<double>("stop_light_brake")),
    m_stop_hard_brake(p.get<double>("stop_hard_brake")),
    m_wait_for_required_signals(p.get<bool>("wait_for_required_signals")),
    m_self_is_disabled_but_requested_actuators_enable(false)
{
  auto dur = p.get_maybe<int>("mission_duration_ms");
  m_mission_duration = dur.has_value()? std::optional<std::chrono::milliseconds>(std::chrono::milliseconds(dur.value())) : std::nullopt;

  if (m_mission_duration.has_value() && m_lap_to_stop.has_value()) {
    RCLCPP_FATAL(logger, "Both mission_duration_ms and lap_to_stop are set!! Aborting!");
    throw std::invalid_argument("Only one between lap_to_stop and mission_duration_ms can be set.");
  }

  if (!m_mission_duration.has_value() && !m_lap_to_stop.has_value()) {
    RCLCPP_FATAL(logger, "Either mission_duration_ms and lap_to_stop must be set!! Aborting!");
    throw std::invalid_argument("Either lap_to_stop or mission_duration_ms must be set.");
  }
}

control::Control EventManager::run_fsm(std::chrono::milliseconds t, const estimation::IVehicleState& x, const control::Control& u, const path::ReferencePath& refpath) {
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
          RCLCPP_INFO(m_logger, "Enabling all actuators and waiting for them to be enabled...");
          m_actuators.request_enable_all();
          m_event_state = EventState::WaitingForActuators;
        }
      }
      // We don't expect any actuator to actuate this...
      return control::Control(0.0, 0.0, m_launch_brake, control::Control::Clutch::Disengaged, 0, control::Control::LaunchControl::Unset);

    case EventState::WaitingForActuators:
      if (m_actuators.all_enabled()) {
        RCLCPP_INFO(m_logger, "Waiting for mandatory signals to be ready...");
        m_event_state = EventState::WaitingForSignals;
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

    case EventState::WaitingForSignals:
      if (!m_wait_for_required_signals) {
        RCLCPP_INFO(m_logger, "Required signals wait is disabled from config!");
      }

      {
        RequiredSignals sig;
        sig.position = x.position().has_value();
        sig.yaw = x.yaw().has_value();
        sig.speed = x.speed().has_value();
        sig.trajectory = refpath.is_valid();

        if (!m_required_signals.has_value() || sig != m_required_signals.value()) {
          RCLCPP_INFO(m_logger, "Required signals availability changed (pos: %s, yaw: %s, speed: %s, trajectory: %s).", sig.position?"T":"F", sig.yaw?"T":"F", sig.speed?"T":"F", sig.trajectory?"T":"F");
          m_required_signals = sig;
        }

        if (sig.all() || !m_wait_for_required_signals) {
          RCLCPP_INFO(m_logger, "Activating Launch Control...");
          m_event_state = EventState::Launch_SetLaunchControl;
        }
      }

      return control::Control(
        0.0,
        0.0,
        m_launch_brake,
        control::Control::Clutch::Disengaged,
        1,
        control::Control::LaunchControl::Unset
      );

    
    case EventState::Launch_SetLaunchControl:
      if (x.lc_is_active().has_value() && x.lc_is_active().value()) {
        RCLCPP_INFO(m_logger, "Revving the engine for %ld milliseconds...", m_rev_duration_before_engage.count());
        m_event_state = EventState::Launch_RevBeforeEngage;
        m_fsm_step_start_time = t;
      }

      return control::Control(
        u.steer,
        0.0,
        m_launch_brake,
        control::Control::Clutch::Disengaged,
        1,
        control::Control::LaunchControl::Set
      );

    case EventState::Launch_RevBeforeEngage:
      if ((t - m_fsm_step_start_time) >= m_rev_duration_before_engage) {
        RCLCPP_INFO(m_logger, "Releasing brakes and engaging clutch. Waiting for the clutch to be engaged...");
        m_event_state = EventState::Launch_EngageClutch;
      }
      return control::Control(
        u.steer,
        m_launch_throttle,
        m_launch_brake,
        control::Control::Clutch::Disengaged,
        1,
        control::Control::LaunchControl::Set
      );

    case EventState::Launch_EngageClutch:
      if (x.clutch_is_engaged().has_value() && x.clutch_is_engaged().value()) {
        RCLCPP_INFO(m_logger, "Revving for %ld more milliseconds...", m_rev_duration_after_engage.count());
        m_event_state = EventState::Launch_RevAfterEngage;
        m_fsm_step_start_time = t;
      }
      return control::Control(
        u.steer,
        m_launch_throttle,
        0.0,
        control::Control::Clutch::Engaged,
        1,
        control::Control::LaunchControl::Set
      );

    case EventState::Launch_RevAfterEngage:
      if ((t - m_fsm_step_start_time) >= m_rev_duration_after_engage) {
        RCLCPP_INFO(m_logger, "Launch finished. Waiting %ld milliseconds before disabling LC...", m_lc_duration_after_launch.count());
        m_fsm_step_start_time = t;
        m_event_state = EventState::Driving_WithLC;
      }
      return control::Control(
        u.steer,
        m_launch_throttle,
        0.0,
        control::Control::Clutch::Engaged,
        1,
        control::Control::LaunchControl::Set
      );
    
    case EventState::Driving_WithLC:
      if ((t - m_fsm_step_start_time) >= m_lc_duration_after_launch) {
        if (m_mission_duration.has_value()) {
          RCLCPP_INFO(m_logger, "Launch finished. Letting the controller drive for %ld milliseconds.", m_mission_duration.value().count());
          m_fsm_step_start_time = t;
        }
        else
          RCLCPP_INFO(m_logger, "Launch finished. Letting the controller drive until lap %d.", m_lap_to_stop.value());

        m_event_state = EventState::Driving;
      }
      {
        control::Control ctrl(u);
        ctrl.launch = control::Control::LaunchControl::Set;
        return ctrl;
      }

    case EventState::Driving:
      assert((m_lap_to_stop.has_value() || m_mission_duration.has_value()) && "Either must be set. This should be checked during initialization.");
      if (
          (m_lap.has_value() && m_lap.value() >= m_lap_to_stop.value()) || 
          (m_mission_duration.has_value() && (t - m_fsm_step_start_time) >= m_mission_duration.value())
         )
      {
        RCLCPP_INFO(m_logger, "Target lap reached. Disengaging clutch.");
        m_event_state = EventState::Stop_DisengageClutch;
      }
      return u;

    case EventState::Stop_DisengageClutch:
      if (x.clutch_is_engaged().has_value() && !x.clutch_is_engaged().value()) {
        RCLCPP_INFO(m_logger, "Clutch disengaged. Braking lightly while waiting for neutral...");
        m_event_state = EventState::Stop_WaitForNeutral;
      }

      return control::Control(
        u.steer,
        0.0,
        0.0,
        control::Control::Clutch::Disengaged,
        0,
        control::Control::LaunchControl::Unset
      );
    
    case EventState::Stop_WaitForNeutral:
      if (x.gear().has_value() && x.gear().value() == 0) {
        RCLCPP_INFO(m_logger, "Gear is neutral. Braking hard until the speed is under %lf m/s.", m_standstill_speed);
        m_event_state = EventState::Stop_Halt;
      }
      return control::Control(
        u.steer,
        0.0,
        m_stop_light_brake,
        control::Control::Clutch::Disengaged,
        0,
        control::Control::LaunchControl::Unset
      );
    
    case EventState::Stop_Halt:
      if (x.speed().has_value() && x.speed().value() <= m_standstill_speed) {
        RCLCPP_INFO(m_logger, "The car speed is below the standstill threshold. Waiting %ld milliseconds while braking just to be sure...", m_standstill_time.count());
        m_event_state = EventState::Stop_EnsureStandstill;
        m_fsm_step_start_time = t;
      }

      return control::Control(
        u.steer,
        0.0,
        m_stop_hard_brake,
        control::Control::Clutch::Engaged,
        0,
        control::Control::LaunchControl::Unset
      );

    case EventState::Stop_EnsureStandstill:
      if (t - m_fsm_step_start_time >= m_standstill_time) {
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
        m_stop_hard_brake,
        control::Control::Clutch::Engaged,
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

control::Control EventManager::tick(std::chrono::nanoseconds t, const estimation::IVehicleState& x, const control::Control& u, const path::ReferencePath& refpath) {
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

    ans = run_fsm(std::chrono::duration_cast<std::chrono::milliseconds>(t), x, u, refpath);
    ++i;
  }

  return ans;
}

}; // namespace event
}; // namespace control_node