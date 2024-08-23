#ifndef CONTROLNODE_PATH_SOURCES_ROS_PATHFROMSPEEDPROFILEPOINTS_HPP
#define CONTROLNODE_PATH_SOURCES_ROS_PATHFROMSPEEDPROFILEPOINTS_HPP

#include <rclcpp/rclcpp.hpp>

#include <mmr_base/msg/speed_profile_points.hpp>

#include <control_node/path/sources/reference_path_source.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

class PathFromSpeedProfilePoints : public ReferencePathSource {
  rclcpp::Subscription<mmr_base::msg::SpeedProfilePoints>::SharedPtr m_sub;

  void msg_cb(std::shared_ptr<const mmr_base::msg::SpeedProfilePoints> msg);

public:
  virtual void init_impl(rclcpp::Node& node, const Parameters& params) override;
};

}; // namespace ros
}; // namespace sources
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_SOURCES_ROS_PATHFROMSPEEDPROFILEPOINTS_HPP