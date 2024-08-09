#include <control_node/viz/viz_manager.hpp>
#include <rclcpp/qos.hpp>

namespace control_node {
namespace viz {

static const rclcpp::QoS QOS = 
  rclcpp::QoS(rclcpp::KeepLast(1))
    .durability_volatile()
    .best_effort();

VizManager::VizManager(rclcpp::Node* node, const Parameters& p, rclcpp::Logger logger)
  : m_logger(logger),
    m_pub(nullptr),
    m_min_interval(std::chrono::milliseconds(p.get<int>("min_interval_ms"))),
    m_last_t(0)
{
  if (p.get<bool>("enabled")) {
    m_pub = node->create_publisher<msgs::MarkerArray>(p.get<std::string>("topic"), QOS);
    RCLCPP_DEBUG(m_logger, "Visualization ENABLED.");
  } else {
    RCLCPP_DEBUG(m_logger, "Visualization DISABLED.");
  }
}

msgs::Marker* VizManager::get_new(int32_t type, float r, float g, float b, float a, const std::string& frame_id)
{
  if (m_pub == nullptr)
    return nullptr;

  auto& m = m_msg.markers.emplace_back();

  m.header.frame_id = frame_id;
  m.ns = "control_node";
  m.id = m_msg.markers.size() - 1;
  m.type = type;
  m.action = msgs::Marker::ADD;
  SET_RGBA(m.color, r, g, b, a);
  SET_XYZW(m.pose.orientation, 0.0, 0.0, 0.0, 1.0);
  SET_XYZ(m.pose.position, 0.0, 0.0, 0.0);
  SET_XYZ(m.scale, 1.0, 1.0, 1.0);
  
  return &m;
}

void VizManager::tick(std::chrono::nanoseconds t) {
  if (m_pub == nullptr)
    return;

  if (t - m_last_t >= m_min_interval) {
    RCLCPP_DEBUG(m_logger, "Publishing visualization.");
    m_last_t = std::chrono::duration_cast<std::chrono::milliseconds>(t);
    m_pub->publish(m_msg);
  }
}

}; // namespace viz
}; // namespace control_node