#ifdef USE_KRIA_MSGS
  #include <mmr_kria_base/msg/marker.hpp>
  #include <mmr_kria_base/msg/marker_array.hpp>
#else
  #include <visualization_msgs/msg/marker.hpp>
  #include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace control_node {
namespace viz_msgs {

#ifdef USE_KRIA_MSGS
  using Marker = mmr_kria_base::msg::Marker;
  using MarkerArray = mmr_kria_base::msg::MarkerArray;
#else
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
#endif

};
};