#ifndef CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP
#define CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP

#include "control_node/path/sources/ros/path_from_marker_msg.hpp"
#include <initializer_list>

namespace control_node {
namespace path {
namespace sources {

namespace _ {
  template <typename T>
  std::unique_ptr<sources::ReferencePathSource> create_source() {
    return std::make_unique<T>();
  }
}

static inline std::unique_ptr<sources::ReferencePathSource> make_path_source(const std::string& name) {
  constexpr static std::initializer_list<std::pair<const char*, std::unique_ptr<sources::ReferencePathSource>(*)()>> SOURCES = {
    std::make_pair("ROS_Marker", _::create_source<sources::ros::PathFromMarkerMsg>),
  };

  for (auto p : SOURCES)
    if (name == p.first)
      return p.second();

  return nullptr;
}

}; // namespace sources
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_SOURCES_SOURCESMAP_HPP