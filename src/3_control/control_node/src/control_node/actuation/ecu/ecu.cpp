#include <control_node/actuation/ecu/ecu.hpp>

namespace control_node {
namespace actuation {
namespace ecu {

void Ecu::init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger) {
  m_pub = node.create_publisher<mmr_base::msg::CmdEcu>(p.get<std::string>("topic"), p.parse_qos("qos"));
}

void Ecu::actuate(std::chrono::nanoseconds, const control::Control& u) {
  if (m_enabled) {
    mmr_base::msg::CmdEcu msg;
    msg.gear_target = u.gear;
    msg.set_launch_control = u.launch == control::Control::LaunchControl::Set;
    m_pub->publish(msg);
  }
}

}; // namespace ecu
}; // namespace actuation
}; // namespace control_node