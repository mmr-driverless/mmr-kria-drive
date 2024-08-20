#ifndef CONTROLNODE_ACTUATION_SIM_SIM_HPP
#define CONTROLNODE_ACTUATION_SIM_SIM_HPP

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>

namespace control_node {
namespace actuation {
namespace sim {

class Sim : public IActuator {
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr m_pub;
  bool m_enabled;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) override;
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& control) override;

  virtual void request_enable() override { m_enabled = true; };
  virtual void request_disable() override { m_enabled = false; };
  virtual bool enabled() const override { return m_enabled; };
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_SIM_SIM_HPP