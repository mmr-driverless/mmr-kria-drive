#ifndef CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP
#define CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP

#include <string>
#include <cstdint>

namespace control_node {
namespace actuation {
namespace spi_apps {

class MCP4921 {
  constexpr static uint8_t CONFIG = 0x3;
  int m_io_device;
  
public:
  constexpr static uint16_t MAX_VALUE = 0xFFF;

  MCP4921(const std::string& interface, uint32_t frequency);
  int write(uint16_t value);
  ~MCP4921();
};

}; // namespace spi_apps
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP