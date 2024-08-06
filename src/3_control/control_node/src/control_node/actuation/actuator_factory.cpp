#include <control_node/actuation/iactuator.hpp>
#include <control_node/actuation/actuator_factory.hpp>

#include <control_node/actuation/canopen_bridge/canopen_bridge.hpp>
#include <control_node/actuation/sim/sim.hpp>

namespace control_node {
namespace actuation {

template <typename T>
static std::unique_ptr<IActuator> create_actuator() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IActuator>(*)()>> ACTUATORS = {
  std::make_pair("CANOpenBridge", create_actuator<canopen_bridge::CANOpenBridge>),
  std::make_pair("Sim", create_actuator<sim::Sim>),
};

static constexpr ComponentFactory<IActuator> FACTORY(ACTUATORS);
const ComponentFactory<IActuator>& get_factory() { return FACTORY; }

}; // namespace actuation
}; // namespace control_node