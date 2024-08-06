#ifndef CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP
#define CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP

#include <control_node/component_factory.hpp>
#include <control_node/path/sources/reference_path_source.hpp>

namespace control_node {
namespace path {
namespace sources {

const ComponentFactory<ReferencePathSource>& get_factory();

}; // namespace sources
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP