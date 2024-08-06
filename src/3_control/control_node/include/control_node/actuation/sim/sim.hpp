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

public:
  virtual void init(rclcpp::Node& node, const Parameters& p) override;
  virtual void actuate(const control::Control& control) override;
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_SIM_SIM_HPP