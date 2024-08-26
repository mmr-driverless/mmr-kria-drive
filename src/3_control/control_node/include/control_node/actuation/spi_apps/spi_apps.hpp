#ifndef CONTROLNODE_ACTUATION_SPIAPPS_SPIAPPS_HPP
#define CONTROLNODE_ACTUATION_SPIAPPS_SPIAPPS_HPP

#include <control_node/actuation/spi_apps/mcp4921.hpp>
#include <control_node/actuation/iactuator.hpp>

namespace control_node {
namespace actuation {
namespace spi_apps {

class SpiApps : public IActuator {
  std::optional<MCP4921> m_device;
  std::optional<rclcpp::Logger> m_logger;

  MCP4921::ConfigFlags m_flags = MCP4921::ConfigFlags::FLAG_ACTIVE | MCP4921::ConfigFlags::FLAG_UNITARY_GAIN;
  bool m_soft_enabled = false;

  double m_v_range;
  double m_v_min;
  double m_v_ref;

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger);
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& control);

  virtual void request_enable() { m_soft_enabled = true; };
  virtual void request_disable() { m_soft_enabled = false; };
  virtual bool enabled() const { return m_device.has_value() && m_soft_enabled; };
};

}; // namespace mcp4921
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_SPIAPPS_SPIAPPS_HPP