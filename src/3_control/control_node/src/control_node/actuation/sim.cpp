#include <control_node/actuation/sim/sim.hpp>

namespace control_node {
namespace actuation {
namespace sim {

void Sim::init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger) {
  m_pub = node.create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(p.get<std::string>("topic"), p.parse_qos("qos"));
}

void Sim::actuate(std::chrono::nanoseconds t, const control::Control& u) {
  if (m_enabled) {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.drive.steering_angle = u.steer;
    if (u.brake > 0) {
      msg.drive.speed = -u.brake;
    } else {
      msg.drive.speed = u.throttle;
    }
    msg.header.stamp = rclcpp::Time(t.count()); 
    m_pub->publish(msg);
  }
}

};
};
};