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
  // TODO: Do i seriously want to pass around a reference to a whole ass Node if i just need to create publishers?
  Sim(rclcpp::Node& node, const Parameters& p);
  ~Sim();
  
  virtual void actuate(const control::Control& control) override;
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_SIM_SIM_HPP