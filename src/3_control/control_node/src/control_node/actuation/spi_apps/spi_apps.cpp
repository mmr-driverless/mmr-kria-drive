#include <control_node/actuation/spi_apps/spi_apps.hpp>
#include <stdexcept>

namespace control_node {
namespace actuation {
namespace spi_apps {

static inline uint16_t voltage_to_value(double v, uint16_t max_value, double vref) {
  v = std::clamp(v, 0.0, vref);
  double value_f = max_value * v / vref;
  int value = std::clamp<int>(static_cast<int>(value_f), 0, max_value);
  return static_cast<uint16_t>(value);
}

SpiApps::SpiApps()
  : m_soft_enabled(false),
    m_v_range(0.0),
    m_v_min(0.0),
    m_v_ref(5.0)  
{ }

void SpiApps::init(rclcpp::Node&, const Parameters& p, rclcpp::Logger logger) {
  auto vref = p.get<double>("v_ref");
  auto vmin = p.get<double>("v_apps_min");
  auto vmax = p.get<double>("v_apps_max");
  auto iface = p.get<std::string>("spi_interface");
  auto freq = p.get<int>("spi_frequency");

  if (vmin < 0) {
    RCLCPP_ERROR(logger, "v_apps_min (%.1lf) must be greater than 0.", vmin);
    throw std::invalid_argument("voltages");
  }
  if (vmax < vmin) {
    RCLCPP_ERROR(logger, "v_apps_max (%.1lf) must be greater than v_apps_min (%.1lf)", vmax, vmin);
    throw std::invalid_argument("voltages");
  }
  if (vref < vmax) {
    RCLCPP_ERROR(logger, "v_ref (%.1lf) must be greater than v_apps_max (%.1lf)", vref, vmin);
    throw std::invalid_argument("voltages");
  }

  m_v_ref = vref;
  m_v_min = vmin;
  m_v_range = vmax - vmin;

  try {
    m_device.emplace(iface, freq);
  } catch(...) {
    RCLCPP_ERROR(logger, "Failed to initialize SPI device @ '%s'!", iface.c_str());
    throw;
  }
  RCLCPP_INFO(logger, "SPI device '%s' successfully initialized!", iface.c_str());
}

void SpiApps::actuate(const control::Control& u) {
  if (!m_device.has_value())
    return;

  double throttle = 0.0;
  if (m_soft_enabled)
    throttle = std::clamp(u.throttle, 0.0, 1.0);

  double voltage = m_v_min + (throttle * m_v_range);
  
  uint16_t data = voltage_to_value(voltage, decltype(m_device)::value_type::MAX_VALUE, m_v_ref);
  if (m_device->write(data) < 0)
    RCLCPP_ERROR(*m_logger, "Could not write APPS 0x%03x: %s", data, strerror(errno));
}

}; // namespace mcp4921
}; // namespace actuation
}; // namespace control_node