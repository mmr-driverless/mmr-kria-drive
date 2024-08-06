#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <span>
#include <array>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <control_node/msg_helpers.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/control_node.hpp>
#include <control_node/viz_msgs.hpp>

using namespace control_node::viz_msgs;

geometry_msgs::msg::Point eigen_vec2_to_msg(Eigen::Vector2d pt) {
  geometry_msgs::msg::Point ans;
  ans.x = pt.x();
  ans.y = pt.y();
  ans.z = 0;
  return ans;
}

static inline Marker create_empty_marker(
  const rclcpp::Time& stamp,
  int id,
  int32_t type,
  std::array<float, 4> color,
  int32_t action = Marker::ADD,
  const std::string& frame_id = "track")
{
  Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "control";
  m.id = id;
  m.type = type;
  m.action = action;
  SET_RGBA(m.color, color[0], color[1], color[2], color[3]);
  SET_XYZW(m.pose.orientation, 0.0, 0.0, 0.0, 1.0);
  SET_XYZ(m.pose.position, 0.0, 0.0, 0.0);
  SET_XYZ(m.scale, 1.0, 1.0, 1.0);
  return m;
}

MarkerArray visualize(
    control_node::ControlNode& node,
    const control_node::path::ReferencePath& path,
    const std::optional<control_node::path::ReferencePath::ProjectionResult> projection,
    Eigen::Vector2d car_position,
    std::optional<Eigen::Vector2d> lookforward,
    double threshold)
{
  auto stamp = node.get_clock()->now();

  MarkerArray msg;

  Eigen::Vector2d closest_position;
  std::array<std::span<Eigen::Vector2d>, 2> waypoints;

  if (projection.has_value()) {
    closest_position = path.get_position(projection->closest_point);
    waypoints = path.get_subpath(projection->first_checked_waypoint, projection->last_checked_waypoint);
  }

  {
    // The threshold area drawn around the vehicle.
    auto m = create_empty_marker(stamp, 0, Marker::CYLINDER, {1.0f, 1.0f, 1.0f, 0.5f});
    SET_XYZ(m.scale, threshold * 2, threshold * 2, 0.1);
    SET_XY(m.pose.position, car_position.x(), car_position.y());
    msg.markers.push_back(m);
  }

  {
    // The closest position.
    auto m = create_empty_marker(stamp, 1, Marker::CYLINDER, {1.0f, 0.0f, 0.0f, projection.has_value()? 1.0f : 0.0f});
    SET_XY(m.pose.position, closest_position.x(), closest_position.y());
    msg.markers.push_back(m);
  }

  {
    // The subpath which was traversed in this iteration
    auto m = create_empty_marker(stamp, 2, Marker::LINE_STRIP, {0.0f, 0.0f, 1.0f, 1.0f});
    SET_XYZ(m.scale, 0.2, 0.2, 0.2);

    for (auto span : waypoints)
      for (auto pt : span)
        m.points.push_back(eigen_vec2_to_msg(pt));

    msg.markers.push_back(m);
  }

  {
    auto m = create_empty_marker(stamp, 3, Marker::CYLINDER, {0.0f, 1.0f, 0.0f, lookforward.has_value()? 1.0f : 0.0f});
    SET_XY(m.pose.position, lookforward->x(), lookforward->y());
    msg.markers.push_back(m);
  }

  return msg;
}

#endif // !VISUALIZE_HPP