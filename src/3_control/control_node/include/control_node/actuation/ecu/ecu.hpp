#ifndef CONTROLNODE_ACTUATION_ECU_ECU_HPP
#define CONTROLNODE_ACTUATION_ECU_ECU_HPP

#include <rclcpp/rclcpp.hpp>

#include <mmr_base/msg/cmd_ecu.hpp>

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace actuation {
namespace ecu {

class Ecu : public IActuator {
  rclcpp::Publisher<mmr_base::msg::CmdEcu>::SharedPtr m_pub;
  bool m_enabled;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) override;
  virtual void actuate(const control::Control& control) override;

  virtual void request_enable() override { m_enabled = true; };
  virtual void request_disable() override { m_enabled = false; };
  virtual bool enabled() const override { return m_enabled; };
};

}; // namespace ecu
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_ECU_ECU_HPP