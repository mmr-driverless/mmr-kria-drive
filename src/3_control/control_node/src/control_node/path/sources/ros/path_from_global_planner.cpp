#include "control_node/path/reference_path.hpp"
#include <control_node/path/sources/ros/path_from_global_planner.hpp>
#include <control_node/parameters.hpp>
#include <cstddef>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

void PathFromGlobalPlanner::msg_cb(std::shared_ptr<const mmr_base::msg::TrajectoryPoints> msg) {
  notifyPathChanged(msg->points.size(), UpdateFn([this, msg](WaypointsT waypoints, ReferencePath::PathData::Data data) {
    for(size_t i = 0; i < msg->points.size(); ++i)
    {
        waypoints[i] = Eigen::Vector2d(msg->points[i].pose.x, msg->points[i].pose.y);
        data.track_yaw[i] = msg->points[i].track_yaw; 
    }

    ReferencePath::PathData::Metadata meta;
    meta.is_track_yaw_valid = true;
    meta.is_from_global_planner = true;
    return UpdateFnResultT(true, meta);
  }));
}

void PathFromGlobalPlanner::init_impl(rclcpp::Node& node, const Parameters& p) {
  m_sub = node.create_subscription<mmr_base::msg::TrajectoryPoints>(p.get<std::string>("topic"), p.parse_qos("qos_override"), std::bind(&PathFromGlobalPlanner::msg_cb, this, std::placeholders::_1));
}

} // namespace ros
} // namespace source
} // namespace path
} // namespace control_node