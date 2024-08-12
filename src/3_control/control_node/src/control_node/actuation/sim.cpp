#include <control_node/actuation/sim/sim.hpp>

namespace control_node {
namespace actuation {
namespace sim {

void Sim::init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger) {
  m_pub = node.create_publisher<ackermann_msgs::msg::AckermannDrive>(p.get<std::string>("topic"), p.parse_qos("qos"));
}

void Sim::actuate(const control::Control& u) {
  if (m_enabled) {
    ackermann_msgs::msg::AckermannDrive msg;
    msg.steering_angle = u.steer;
    m_pub->publish(msg);
  }
}

};
};
};