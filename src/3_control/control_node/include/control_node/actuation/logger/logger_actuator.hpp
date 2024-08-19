#ifndef CONTROLNODE_ACTUATION_LOGGER_LOGGERACTUATOR_HPP
#define CONTROLNODE_ACTUATION_LOGGER_LOGGERACTUATOR_HPP

#include "control_node/control/control.hpp"
#include <mmr_base/msg/detail/control_log__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mmr_base/msg/control_log.hpp>

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace actuation {
namespace logger {

class LoggerActuator : public IActuator {
  rclcpp::Publisher<mmr_base::msg::ControlLog>::SharedPtr m_pub;
  bool m_enabled;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger) override {
    m_pub = node.create_publisher<mmr_base::msg::ControlLog>(p.get<std::string>("topic"), p.parse_qos("qos"));
  }
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& u) override {
    mmr_base::msg::ControlLog msg;
    msg.header.frame_id = "suca";
    msg.header.stamp = rclcpp::Time(t.count());
    msg.brake = u.brake;
    msg.clutch = u.clutch == control::Control::Clutch::Engaged;
    msg.launch = u.launch == control::Control::LaunchControl::Set;
    msg.gear = u.gear;
    msg.steer = u.steer;
    msg.throttle = u.throttle;
    m_pub->publish(msg);
  }

  virtual void request_enable() override { m_enabled = true; };
  virtual void request_disable() override { m_enabled = false; };
  virtual bool enabled() const override { return m_enabled; };
};

}; // namespace logger
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_LOGGER_LOGGERACTUATOR_HPP