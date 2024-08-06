#ifndef CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP
#define CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <control_node/parameters.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/path/sources/reference_path_source.hpp>
#include <vector>

namespace control_node {
namespace path {

class ReferencePathManager {
  std::vector<Eigen::Vector2d> m_waypoints;
  std::vector<ReferencePath::PointData::StorageT> m_data;

  std::vector<std::pair<int, std::unique_ptr<sources::ReferencePathSource>>> m_sources;
  int m_max_activated_source_idx;

  bool m_changed;

  ReferencePath m_path;

  rclcpp::Logger m_logger;

  void on_source_notification(int source_id, size_t sz, const sources::ReferencePathSource::UpdateFn& ufn);
public:
  ReferencePathManager(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger);

  bool changed();
  const ReferencePath& get() const;
};

}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP