#include <control_node/actuation/sim/sim.hpp>

namespace control_node {
namespace actuation {
namespace sim {

Sim::Sim(rclcpp::Node& node, const Parameters& p)
  : m_pub(node.create_publisher<ackermann_msgs::msg::AckermannDrive>(p.get<std::string>("topic"), 1))
{}

void Sim::actuate(const control::Control& u) {
  ackermann_msgs::msg::AckermannDrive msg;
  msg.steering_angle = u.steer;
  m_pub->publish(msg);
}

};
};
};