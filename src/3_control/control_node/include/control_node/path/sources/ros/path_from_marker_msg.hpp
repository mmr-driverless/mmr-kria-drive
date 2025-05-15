#ifndef CONTROLNODE_PATH_SOURCES_ROS_PATHFROMMARKERMSG_HPP
#define CONTROLNODE_PATH_SOURCES_ROS_PATHFROMMARKERMSG_HPP

#include <control_node/viz/msgs/viz_msgs.hpp>
#include <control_node/path/sources/reference_path_source.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

class PathFromMarkerMsg : public ReferencePathSource {
  rclcpp::Subscription<viz::msgs::Marker>::SharedPtr m_sub;
  bool m_is_closed;

  void msg_cb(std::shared_ptr<const viz::msgs::Marker> msg);

public:
  PathFromMarkerMsg() {}
  virtual void init_impl(rclcpp::Node& node, const Parameters& params) override;
};

} // namespace ros
} // namespace sources
} // namespace path
} // namespace control_node

#endif // !CONTROLNODE_PATH_SOURCES_ROS_PATHFROMMARKERMSG_HPP