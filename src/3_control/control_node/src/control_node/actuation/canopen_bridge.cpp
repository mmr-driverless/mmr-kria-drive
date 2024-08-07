#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/configuration.hpp>

namespace control_node {
namespace actuation {
namespace canopen_bridge {

CANOpenBridge::~CANOpenBridge() {
  request_disable();
  serve_enable_request();
}

void CANOpenBridge::serve_enable_request() {
  mmr_base::msg::CmdMotor cmd_msg;

  if (m_enable_request == EnableRequest::Enable) {
    if (m_status.clutch.is_enabled() && m_status.brake.is_enabled() && m_status.steer.is_enabled()) {
      RCLCPP_INFO(logger(), "Motors successfully enabled!");
      m_enable_request = EnableRequest::None;
      return;
    }

    if (m_status.steer.is_disabled()) {
      cmd_msg.enable = false;
      cmd_msg.disable = false;
      cmd_msg.homing = true;
      m_steer_pub->publish(cmd_msg);

      cmd_msg.enable = true;
      cmd_msg.disable = false;
      cmd_msg.homing = false;
      m_steer_pub->publish(cmd_msg);
    }

    cmd_msg.enable = true;
    cmd_msg.disable = false;
    cmd_msg.homing = false;

    if (m_status.brake.is_disabled())
      m_brake_pub->publish(cmd_msg);
    if (m_status.clutch.is_disabled())
      m_clutch_pub->publish(cmd_msg);

  } else if (m_enable_request == EnableRequest::Disable) {
    if (m_status.clutch.is_disabled() && m_status.brake.is_disabled() && m_status.steer.is_disabled()) {
      RCLCPP_INFO(logger(), "Motors successfully disabled!");
      return;
    }

    cmd_msg.enable = false;
    cmd_msg.homing = false;
    cmd_msg.disable = true;
    
    if (m_status.steer.is_enabled())
      m_steer_pub->publish(cmd_msg);
    if (m_status.brake.is_enabled())
      m_brake_pub->publish(cmd_msg);
    if (m_status.clutch.is_enabled())
      m_clutch_pub->publish(cmd_msg);
  }
}

void CANOpenBridge::init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) {
  auto pub_qos = p.parse_qos("pub_qos_override");

  m_logger = logger;
  m_steer_pub = node.create_publisher<mmr_base::msg::CmdMotor>(p.get<std::string>("steer_topic"), pub_qos);
  m_brake_pub = node.create_publisher<mmr_base::msg::CmdMotor>(p.get<std::string>("brake_topic"), pub_qos);
  m_clutch_pub = node.create_publisher<mmr_base::msg::CmdMotor>(p.get<std::string>("clutch_topic"), pub_qos);
  m_act_status_sub = node.create_subscription<mmr_base::msg::ActuatorStatus>(p.get<std::string>("actuator_status_topic"), p.parse_qos("actuator_status_qos_override"), std::bind(&CANOpenBridge::actuator_status_cb, this, std::placeholders::_1));

  RCLCPP_INFO(this->logger(), "Initialized");
}

void CANOpenBridge::actuator_status_cb(mmr_base::msg::ActuatorStatus::SharedPtr msg) {
  m_status.steer = (MOTOR::ACTUATOR_STATUS)msg->steer_status == MOTOR::ACTUATOR_STATUS::DISABLE? ActuatorStatus::Disabled : ActuatorStatus::Enabled;
  m_status.brake = (MOTOR::ACTUATOR_STATUS)msg->brake_status == MOTOR::ACTUATOR_STATUS::DISABLE? ActuatorStatus::Disabled : ActuatorStatus::Enabled;
  m_status.clutch = (MOTOR::ACTUATOR_STATUS)msg->clutch_status == MOTOR::ACTUATOR_STATUS::DISABLE? ActuatorStatus::Disabled : ActuatorStatus::Enabled;

  if (!m_old_status.has_value() || (m_old_status.value() != m_status))
    RCLCPP_INFO(logger(), "Actuator status changed (Steer: %s, Brake: %s, Clutch: %s)", m_status.steer.to_string(), m_status.brake.to_string(), m_status.clutch.to_string());

  m_old_status = m_status;

  serve_enable_request();
}

void CANOpenBridge::actuate(const control::Control& u) {
  if (m_status.steer.is_enabled()) {
    mmr_base::msg::CmdMotor msg;
    msg.wheel_angle = u.steer;
    m_steer_pub->publish(msg);
  }
  if (m_status.brake.is_enabled()) {
    mmr_base::msg::CmdMotor msg;
    msg.brake_torque = u.brake;
    m_brake_pub->publish(msg);
  }
  if (m_status.clutch.is_enabled()) {
    mmr_base::msg::CmdMotor msg;
    msg.disengaged = (u.clutch == control::Control::Clutch::Disengaged);
    m_steer_pub->publish(msg);
  }
}

bool CANOpenBridge::enabled() const {
  return m_status.steer.is_enabled() && m_status.clutch.is_enabled() && m_status.brake.is_enabled();
}

void CANOpenBridge::request_disable() {
  m_enable_request = EnableRequest::Disable;
  RCLCPP_INFO(logger(), "Received disable request.");
}
void CANOpenBridge::request_enable() {
  m_enable_request = EnableRequest::Enable;
  RCLCPP_INFO(logger(), "Received enable request.");
}

}; // namespace canopen_bridge
}; // namespace actuation
}; // namespace control_node