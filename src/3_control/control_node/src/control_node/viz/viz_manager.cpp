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
    m_last_t(0),
    m_is_viz_tick(false)
{
  if (p.get<bool>("enabled")) {
    m_pub = node->create_publisher<msgs::MarkerArray>(p.get<std::string>("topic"), QOS);
    RCLCPP_DEBUG(m_logger, "Visualization ENABLED.");
  } else {
    RCLCPP_DEBUG(m_logger, "Visualization DISABLED.");
  }
}

int VizManager::get_new(int32_t type, std::array<float, 3> color, std::array<double, 3> scale, const std::string& frame_id)
{
  if (m_pub == nullptr)
    return -1;

  int ans = m_msg.markers.size();
  viz::msgs::Marker& m = m_msg.markers.emplace_back();

  m.header.frame_id = frame_id;
  m.ns = "control_node";
  m.id = m_msg.markers.size() - 1;
  m.type = type;
  m.action = msgs::Marker::ADD;
  SET_RGBA(m.color, color[0], color[1], color[2], 0.0f);
  SET_XYZW(m.pose.orientation, 0.0, 0.0, 0.0, 1.0);
  SET_XYZ(m.pose.position, 0.0, 0.0, 0.0);
  SET_XYZ(m.scale, scale[0], scale[1], scale[2]);

  return ans;
}

void VizManager::pre_tick(std::chrono::nanoseconds t_ns) {
  if (m_pub == nullptr) {
    m_is_viz_tick = false;
    return;
  }

  auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_ns);
  m_is_viz_tick = t - m_last_t >= m_min_interval;
  if (m_is_viz_tick)
    m_last_t = t;
}

void VizManager::tick() {
  if (!m_is_viz_tick)
    return;

  RCLCPP_DEBUG(m_logger, "Publishing visualization.");
  m_pub->publish(m_msg);
}

}; // namespace viz
}; // namespace control_node