#include <control_node/viz/viz_manager.hpp>
#include <rclcpp/qos.hpp>

namespace control_node {
namespace viz {

static const rclcpp::QoS QOS = 
  rclcpp::QoS(rclcpp::KeepLast(1))
    .durability_volatile()
    .best_effort();

VizManager::VizManager(rclcpp::Node* node, const Parameters& p)
{
  m_enabled = p.get<bool>("enabled");
  m_min_interval = std::chrono::milliseconds(p.get<int>("min_interval_ms"));
  m_pub = node->create_publisher<msgs::MarkerArray>(p.get<std::string>("topic"), QOS);
}

msgs::Marker* VizManager::get_new(int32_t type, float r, float g, float b, float a, const std::string& frame_id)
{
  m_msg.markers.emplace_back();
  auto& m = m_msg.markers.back();

  m.header.frame_id = frame_id;
  m.ns = "control_node";
  m.id = m_msg.markers.size() - 1;
  m.type = type;
  m.action = msgs::Marker::ADD;
  SET_RGBA(m.color, r, g, b, a);
  SET_XYZW(m.pose.orientation, 0.0, 0.0, 0.0, 1.0);
  SET_XYZ(m.pose.position, 0.0, 0.0, 0.0);
  SET_XYZ(m.scale, 1.0, 1.0, 1.0);
  
  return &m_msg.markers.back();
}

void VizManager::tick(std::chrono::milliseconds t) {
  if (!m_enabled)
    return;

  if (t - m_last_t > m_min_interval) {
    m_last_t = t;
    m_pub->publish(m_msg);
  }
}

}; // namespace viz
}; // namespace control_node