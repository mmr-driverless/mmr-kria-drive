#ifndef CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP
#define CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mmr_base/msg/cmd_motor.hpp>

namespace control_node {
namespace actuation {
namespace canopen_bridge {

class CANOpenBridge : public IActuator {
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr m_steer_pub;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p) override;
  virtual void actuate(const control::Control& control) override;
  ~CANOpenBridge();
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP