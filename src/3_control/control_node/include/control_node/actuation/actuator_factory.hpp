#ifndef CONTROLNODE_ACTUATION_ACTUATORFACTORY_HPP
#define CONTROLNODE_ACTUATION_ACTUATORFACTORY_HPP

#include <control_node/actuation/iactuator.hpp>
#include <control_node/component_factory.hpp>

namespace control_node {
namespace actuation {

const ComponentFactory<IActuator>& get_factory();

}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_ACTUATORFACTORY_HPP