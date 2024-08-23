#include <control_node/path/sources/ros/path_from_speed_profile_points.hpp>

namespace control_node {
namespace path {
namespace sources {
namespace ros {

void PathFromSpeedProfilePoints::init_impl(rclcpp::Node& node, const Parameters& params) {
  m_sub = node.create_subscription<mmr_base::msg::SpeedProfilePoints>(params.get<std::string>("topic"), params.parse_qos("qos_override"), std::bind(&PathFromSpeedProfilePoints::msg_cb, this, std::placeholders::_1));
}

void PathFromSpeedProfilePoints::msg_cb(std::shared_ptr<const mmr_base::msg::SpeedProfilePoints> msg) {
  notifyPathChanged(msg->points.size(), UpdateFn([&msg](WaypointsT waypoints, ReferencePath::PathData::Data data) {
    for (size_t i = 0; i < msg->points.size(); i++) {
      const mmr_base::msg::SpeedProfilePoint& pt = msg->points[i];

      waypoints[i] = Eigen::Vector2d(pt.point.x, pt.point.y);
      data.target_speed[i] = pt.ackerman_point.speed;
      // TODO: where the fuck is curvature
    }

    ReferencePath::PathData::Metadata meta;
    meta.is_target_speed_valid = true;
    return UpdateFnResultT(true, meta);
  }));
}

}; // namespace ros
}; // namespace sources
}; // namespace path
}; // namespace control_node