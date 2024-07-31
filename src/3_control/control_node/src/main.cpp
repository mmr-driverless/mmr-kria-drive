#include <trajectory.hpp>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ControlNode : public rclcpp::Node {
  ReferencePath m_path;

  ReferencePath::WaypointRef m_progress_ref;

  std::vector<WaypointT> m_waypoints;
  std::vector<DataT> m_path_data;

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
    m_track_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/control/viz", 1);
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
      m_waypoints[i] = WaypointT(msg->points[i].x, msg->points[i].y);

    m_path = ReferencePath(std::span<WaypointT>(m_waypoints), std::span<DataT>(m_path_data));
  }

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg) {
    WaypointT car_position = WaypointT(msg->pose.pose.position.x, msg->pose.pose.position.y);

    if (auto projection = m_path.project_vehicle(car_position, m_progress_ref)) {
      m_progress_ref = projection->window_start;

      std::cout << "Start: " << projection->window_start.get() << ", Target: " << projection->track_point.get() << std::endl;

      WaypointT start_position = m_path.get_position(projection->window_start);
      WaypointT track_position = m_path.get_position(projection->track_point);

      visualize(start_position, track_position, car_position);
    } else
      RCLCPP_WARN(this->get_logger(), "No position");
  }

  geometry_msgs::msg::Point eigen_to_geometry_msgs(WaypointT pt) {
    geometry_msgs::msg::Point ans;
    ans.x = pt.x();
    ans.y = pt.y();
    ans.z = 0;
    return ans;
  }

  void visualize(WaypointT start_position, WaypointT track_position, WaypointT car_position) {
    geometry_msgs::msg::Point start_point = eigen_to_geometry_msgs(start_position);
    geometry_msgs::msg::Point track_point = eigen_to_geometry_msgs(track_position);
    geometry_msgs::msg::Point car_point = eigen_to_geometry_msgs(car_position);
    
    visualization_msgs::msg::MarkerArray msg;

    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "track";
      m.header.stamp = this->get_clock()->now();
      m.ns = "control";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 1.0;
      m.scale.y = 1.0;
      m.scale.z = 0.1;
      m.pose.position = track_point;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;

      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;

      msg.markers.push_back(m);
    }

    m_track_pub->publish(marker);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
}