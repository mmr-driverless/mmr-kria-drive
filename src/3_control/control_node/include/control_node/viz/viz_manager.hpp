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
  rclcpp::Logger m_logger;
  rclcpp::Publisher<msgs::MarkerArray>::SharedPtr m_pub;
  std::chrono::milliseconds m_min_interval;
  std::chrono::milliseconds m_last_t;

public:
  VizManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger);

  void tick(std::chrono::milliseconds t);
  msgs::Marker* get_new(int32_t type, float r, float g, float b, float a, const std::string& frame_id = "track");
};

}; // namespace viz
}; // namespace control_node

#endif // !CONTROLNODE_VIZ_VIZMANAGER_HPP