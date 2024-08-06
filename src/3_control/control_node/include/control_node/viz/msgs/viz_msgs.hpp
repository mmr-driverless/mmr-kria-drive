#ifndef CONTROLNODE_VIZ_MSGS_VIZMSGS_HPP
#define CONTROLNODE_VIZ_MSGS_VIZMSGS_HPP

#ifdef USE_KRIA_MSGS
  #include <mmr_base/msg/marker.hpp>
  #include <mmr_base/msg/marker_array.hpp>
#else
  #include <visualization_msgs/msg/marker.hpp>
  #include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace control_node {
namespace viz {
namespace msgs {

#ifdef USE_KRIA_MSGS
  using Marker = mmr_base::msg::Marker;
  using MarkerArray = mmr_base::msg::MarkerArray;
#else
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
#endif

}; // namespace msgs
}; // namespace viz
}; // namespace control_node

#endif // !CONTROLNODE_VIZ_MSGS_VIZMSGS_HPP