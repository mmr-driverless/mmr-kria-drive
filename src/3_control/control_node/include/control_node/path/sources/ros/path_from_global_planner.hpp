#ifndef CONTROLNODE_PATH_SOURCES_ROS_PATHFROMGLOBALPLANNER_HPP
#define CONTROLNODE_PATH_SOURCES_ROS_PATHFROMGLOBALPLANNER_HPP

#include <control_node/viz/msgs/viz_msgs.hpp>
#include <mmr_base/msg/speed_profile_points.hpp>
#include <control_node/path/sources/reference_path_source.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

class PathFromGlobalPlanner : public ReferencePathSource {
  rclcpp::Subscription<mmr_base::msg::SpeedProfilePoints>::SharedPtr m_sub;

  void msg_cb(std::shared_ptr<const mmr_base::msg::SpeedProfilePoints> msg);

public:
  PathFromGlobalPlanner() {}
  virtual void init_impl(rclcpp::Node& node, const Parameters& params) override;
};

} // namespace ros
} // namespace sources
} // namespace path
} // namespace control_node

#endif // !CONTROLNODE_PATH_SOURCES_ROS_PATHFROMGLOBALPLANNER_HPP