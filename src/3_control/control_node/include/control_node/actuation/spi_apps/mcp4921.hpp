#ifndef CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP
#define CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP

#include <string>
#include <cstdint>

namespace control_node {
namespace actuation {
namespace spi_apps {

class MCP4921 {
  uint8_t m_config;
  int m_io_device;
  
public:
  constexpr static uint16_t MAX_VALUE = 0xFFF;

  enum class ConfigFlags {
    // The device is active
    FLAG_ACTIVE = 0x1,

    // The device has unitary gain (2x otherwise)
    FLAG_UNITARY_GAIN = 0x2,

    // VRef is buffered
    FLAG_BUFFERED = 0x4
  };

  MCP4921(const std::string& interface, uint32_t frequency);
  int write(uint16_t value, ConfigFlags flags);
  ~MCP4921();
};

inline MCP4921::ConfigFlags operator|(MCP4921::ConfigFlags a, MCP4921::ConfigFlags b) {
  return (MCP4921::ConfigFlags)(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

}; // namespace spi_apps
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_SPIAPPS_MCP4921_HPP