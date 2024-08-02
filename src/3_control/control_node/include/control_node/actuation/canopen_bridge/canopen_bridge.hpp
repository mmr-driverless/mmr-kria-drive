#ifndef CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP
#define CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mmr_kria_base/msg/cmd_motor.hpp>

namespace control_node {
namespace actuation {
namespace canopen_bridge {

class CANOpenBridge : public IActuator {
  rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr m_steer_pub;

public:
  // TODO: Do i seriously want to pass around a reference to a whole ass Node if i just need to create publishers?
  CANOpenBridge(rclcpp::Node& node, const Parameters& p);
  ~CANOpenBridge();
  
  virtual void actuate(const control::Control& control) override;
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP