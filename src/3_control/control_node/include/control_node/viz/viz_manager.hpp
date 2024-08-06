#ifndef CONTROLNODE_VIZ_VIZMANAGER_HPP
#define CONTROLNODE_VIZ_VIZMANAGER_HPP


#include <rclcpp/rclcpp.hpp>
#include <control_node/parameters.hpp>
#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/viz/msgs/msg_helpers.hpp>

namespace control_node {
namespace viz {

class VizManager {
  msgs::MarkerArray m_msg;
  rclcpp::Publisher<msgs::MarkerArray>::SharedPtr m_pub;
  std::chrono::milliseconds m_min_interval;
  std::chrono::milliseconds m_last_t;
  bool m_enabled;

public:
  VizManager(rclcpp::Node* node, const Parameters& p);

  void tick(std::chrono::milliseconds t);
  msgs::Marker& get_new(int32_t type, std::array<float, 4> color, const std::string& frame_id = "track");
};

}; // namespace viz
}; // namespace control_node

#endif // !CONTROLNODE_VIZ_VIZMANAGER_HPP