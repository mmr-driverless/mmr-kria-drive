#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>

namespace control_node {
namespace actuation {
namespace canopen_bridge {

CANOpenBridge::CANOpenBridge(rclcpp::Node& node, const Parameters& p)
  : m_steer_pub(node.create_publisher<mmr_base::msg::CmdMotor>(p.get<std::string>("steer_topic"), 2))
{
  // TODO: Enabling the motors here is probably wrong, but whatever
  mmr_base::msg::CmdMotor msg;
  msg.homing = true;
  m_steer_pub->publish(msg);
  msg.homing = false;
  msg.enable = true;
  m_steer_pub->publish(msg);
}

CANOpenBridge::~CANOpenBridge() {
  // TODO: Disabling the motors here is probably wrong, but whatever
  mmr_base::msg::CmdMotor msg;
  msg.enable = false;
  m_steer_pub->publish(msg);
}

void CANOpenBridge::actuate(const control::Control& u) {
  mmr_base::msg::CmdMotor msg;
  msg.wheel_angle = u.steer;
  m_steer_pub->publish(msg);
}

};
};
};