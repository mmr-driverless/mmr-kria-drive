#include <reference_path.hpp>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "msg_helpers.hpp"

static constexpr double THRESHOLD = 4.0 * 4.0;

class ControlNode : public rclcpp::Node {
  ReferencePath m_path;

  std::vector<PointT> m_waypoints;
  std::vector<DataT> m_path_data;

  std::optional<ReferencePath::PointRef> m_last_path_ref;

  bool m_ignore_path_updates;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_viz_pub;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_centerline_cmpl_sub;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr m_centerline_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

public:
  ControlNode() : rclcpp::Node("control_node"), m_ignore_path_updates(false) {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 1, std::bind(&ControlNode::odom_cb, this, std::placeholders::_1));
    m_centerline_sub = this->create_subscription<visualization_msgs::msg::Marker>("/planning/center_line", 1, std::bind(&ControlNode::centerline_cb, this, std::placeholders::_1));
    m_centerline_cmpl_sub = this->create_subscription<visualization_msgs::msg::Marker>("/planning/center_line_completed", qos, std::bind(&ControlNode::centerline_completed_cb, this, std::placeholders::_1));
    m_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/control/viz", 2);
  }

  void centerline_completed_cb(visualization_msgs::msg::Marker::SharedPtr msg) { path_cb(true, msg); }
  void centerline_cb(visualization_msgs::msg::Marker::SharedPtr msg) { path_cb(false, msg); }

  void path_cb(bool completed, visualization_msgs::msg::Marker::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "PATH UPDATE");

    if (m_ignore_path_updates)
      return;

    if (completed) {
      RCLCPP_INFO(this->get_logger(), "Path finalized with completed centerline.");
      m_ignore_path_updates = true;
    }
    
    m_waypoints.resize(msg->points.size());
    m_path_data.resize(msg->points.size());

    RCLCPP_INFO(this->get_logger(), "Path updated %zu", msg->points.size());

    for (size_t i = 0; i < msg->points.size(); ++i)
      m_waypoints[i] = PointT(msg->points[i].x, msg->points[i].y);
    m_last_path_ref = std::nullopt;
    m_path = ReferencePath(std::span<PointT>(m_waypoints), std::span<DataT>(m_path_data), completed);
  }

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg) {
    PointT car_position = PointT(msg->pose.pose.position.x, msg->pose.pose.position.y);
    auto projection = m_path.project_vehicle(car_position, m_last_path_ref, THRESHOLD);

    if (projection) {
      m_last_path_ref = projection->closest_point;
      std::cout << projection->first_checked_waypoint.get_waypoint_idx() << " : " << projection->last_checked_waypoint.get_waypoint_idx() << "(" << m_waypoints.size() << ")" << std::endl;
    }
    else
      RCLCPP_WARN(this->get_logger(), "No position");

    std::optional<PointT> lookforward;
    if (projection) {
      ReferencePath::PointRef r = m_path.advance_point(projection->closest_point, 5);
      lookforward = m_path.get_position(r);
    }

    visualize(projection, car_position, lookforward);
  }

  geometry_msgs::msg::Point eigen_vec2_to_msg(PointT pt) {
    geometry_msgs::msg::Point ans;
    ans.x = pt.x();
    ans.y = pt.y();
    ans.z = 0;
    return ans;
  }

  static inline visualization_msgs::msg::Marker create_empty_marker(
    const rclcpp::Time& stamp,
    int id,
    int32_t type,
    std::array<float, 4> color,
    int32_t action = visualization_msgs::msg::Marker::ADD,
    const std::string& frame_id = "track")
  {
    visualization_msgs::msg::Marker m;
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

  void visualize(std::optional<ReferencePath::ProjectionResult> projection, PointT car_position, std::optional<PointT> lookforward) {
    auto stamp = this->get_clock()->now();

    visualization_msgs::msg::MarkerArray msg;

    PointT closest_position;
    std::array<std::span<PointT>, 2> waypoints;

    if (projection.has_value()) {
      closest_position = m_path.get_position(projection->closest_point);
      waypoints = m_path.get_subpath(projection->first_checked_waypoint, projection->last_checked_waypoint);
    }

    {
      // The threshold area drawn around the vehicle.
      auto m = create_empty_marker(stamp, 0, visualization_msgs::msg::Marker::CYLINDER, {1.0f, 1.0f, 1.0f, 0.5f});
      SET_XYZ(m.scale, THRESHOLD / 2, THRESHOLD / 2, 0.1);
      SET_XY(m.pose.position, car_position.x(), car_position.y());
      msg.markers.push_back(m);
    }

    {
      // The closest position.
      auto m = create_empty_marker(stamp, 1, visualization_msgs::msg::Marker::CYLINDER, {1.0f, 0.0f, 0.0f, projection.has_value()? 1.0f : 0.0f});
      SET_XY(m.pose.position, closest_position.x(), closest_position.y());
      msg.markers.push_back(m);
    }

    {
      // The subpath which was traversed in this iteration
      auto m = create_empty_marker(stamp, 2, visualization_msgs::msg::Marker::LINE_STRIP, {0.0f, 0.0f, 1.0f, 1.0f});
      SET_XYZ(m.scale, 0.2, 0.2, 0.2);

      for (auto span : waypoints)
        for (auto pt : span)
          m.points.push_back(eigen_vec2_to_msg(pt));

      msg.markers.push_back(m);
    }

    {
      auto m = create_empty_marker(stamp, 3, visualization_msgs::msg::Marker::CYLINDER, {0.0f, 1.0f, 0.0f, lookforward.has_value()? 1.0f : 0.0f});
      SET_XY(m.pose.position, lookforward->x(), lookforward->y());
      msg.markers.push_back(m);
    }

    m_viz_pub->publish(msg);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
}