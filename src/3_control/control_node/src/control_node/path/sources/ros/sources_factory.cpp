#include <control_node/path/sources/sources_factory.hpp>

#include <control_node/path/sources/ros/path_from_marker_msg.hpp>

namespace control_node {
namespace path {
namespace sources {

template <typename T>
static std::unique_ptr<ReferencePathSource> create_source() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<ReferencePathSource>(*)()>> SOURCES = {
  std::make_pair("ROS_Marker", create_source<ros::PathFromMarkerMsg>),
};

static constexpr ComponentFactory<ReferencePathSource> FACTORY(SOURCES);
const ComponentFactory<ReferencePathSource>& get_factory() { return FACTORY; }

}; // namespace sources
}; // namespace path
}; // namespace control_node