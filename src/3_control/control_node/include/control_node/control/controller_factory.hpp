#ifndef CONTROLNODE_CONTROL_CONTROLLERFACTORY_HPP
#define CONTROLNODE_CONTROL_CONTROLLERFACTORY_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/component_factory.hpp>

namespace control_node {
namespace control {

const ComponentFactory<IController>& get_factory();

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROLLERFACTORY_HPP