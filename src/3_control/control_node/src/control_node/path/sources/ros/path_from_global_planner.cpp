#include "control_node/path/reference_path.hpp"
#include <control_node/path/sources/ros/path_from_global_planner.hpp>
#include <control_node/parameters.hpp>
#include <cstddef>
#include <mmr_base/msg/detail/speed_profile_points__struct.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

void PathFromGlobalPlanner::msg_cb(std::shared_ptr<const mmr_base::msg::SpeedProfilePoints> msg) {
  notifyPathChanged(msg->points.size(), UpdateFn([this, msg](WaypointsT waypoints, ReferencePath::PathData::Data data) {
    for(size_t i = 0; i < msg->points.size(); ++i)
    {
        waypoints[i] = Eigen::Vector2d(msg->points[i].point.x, msg->points[i].point.y);
        // data.curvature[i] = msg->points[i].ackerman_point.curvature; 
    }

    ReferencePath::PathData::Metadata meta;
    // meta.is_curvature_valid = true;
    return UpdateFnResultT(true, meta);
  }));
}

void PathFromGlobalPlanner::init_impl(rclcpp::Node& node, const Parameters& p) {
  m_sub = node.create_subscription<mmr_base::msg::SpeedProfilePoints>(p.get<std::string>("topic"), p.parse_qos("qos_override"), std::bind(&PathFromGlobalPlanner::msg_cb, this, std::placeholders::_1));
}

} // namespace ros
} // namespace source
} // namespace path
} // namespace control_node