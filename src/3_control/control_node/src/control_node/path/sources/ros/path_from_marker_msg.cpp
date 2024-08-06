#include <control_node/path/sources/ros/path_from_marker_msg.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

void PathFromMarkerMsg::msg_cb(viz::msgs::Marker::SharedPtr msg) {
  notifyPathChanged(msg->points.size(), UpdateFn([this, &msg](WaypointsT waypoints, DataT _) {
    for (size_t i = 0; i < msg->points.size(); i++)
      waypoints[i] = Eigen::Vector2d(msg->points[i].x, msg->points[i].y);
    
    return m_path_prop;
  }));
}

void PathFromMarkerMsg::init_impl(rclcpp::Node& node, const Parameters& p) {
  m_path_prop.is_closed = p.get<bool>("is_closed");
  m_path_prop.is_data_valid = false;
  m_sub = node.create_subscription<viz::msgs::Marker>(p.get<std::string>("topic"), p.parse_qos("qos_override"), std::bind(&PathFromMarkerMsg::msg_cb, this, std::placeholders::_1));
}

}; // namespace ros
}; // namespace source
}; // namespace path
}; // namespace control_node