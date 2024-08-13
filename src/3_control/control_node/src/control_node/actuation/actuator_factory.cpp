#include "control_node/actuation/ecu/ecu.hpp"
#include <control_node/actuation/iactuator.hpp>
#include <control_node/actuation/actuator_factory.hpp>

#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>
#include <control_node/actuation/spi_apps/spi_apps.hpp>
#include <control_node/actuation/sim/sim.hpp>

namespace control_node {
namespace actuation {

template <typename T>
static std::unique_ptr<IActuator> create_actuator() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IActuator>(*)()>> ACTUATORS = {
  std::make_pair("CANOpenBridge", create_actuator<canopen_bridge::CANOpenBridge>),
  std::make_pair("Sim", create_actuator<sim::Sim>),
  std::make_pair("SpiApps", create_actuator<spi_apps::SpiApps>),
  std::make_pair("Ecu", create_actuator<ecu::Ecu>)
};

static constexpr ComponentFactory<IActuator> FACTORY(ACTUATORS);
const ComponentFactory<IActuator>& get_factory() { return FACTORY; }

}; // namespace actuation
}; // namespace control_node