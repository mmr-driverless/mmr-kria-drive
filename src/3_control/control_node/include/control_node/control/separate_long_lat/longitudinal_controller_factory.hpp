#ifndef CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROLLERFACTORY_HPP
#define CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROLLERFACTORY_HPP

#include <control_node/control/separate_long_lat/ilongitudinal_controller.hpp>
#include <control_node/component_factory.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

const ComponentFactory<ILongitudinalController>& get_longitudinal_controller_factory();

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_SEPARATELONGLAT_LONGITUDINALCONTROLLERFACTORY_HPP