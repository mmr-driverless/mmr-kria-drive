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

  bool m_is_viz_tick;

public:
  VizManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger);

  void pre_tick(std::chrono::nanoseconds t);
  void tick();

  int get_new(int32_t type, std::array<float, 3> color, std::array<double, 3> scale = { 1.0, 1.0, 1.0 }, const std::string& frame_id = "track");
  msgs::Marker& get(int id) { return m_msg.markers.at(id); }
  msgs::Marker* get_if_viz_tick(int id) {
    return m_is_viz_tick? &get(id) : nullptr;
  }
  bool is_viz_tick() const { return m_is_viz_tick; }
};

}; // namespace viz
}; // namespace control_node

#endif // !CONTROLNODE_VIZ_VIZMANAGER_HPP