#ifndef CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP
#define CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP

#include <vector>
#include <filesystem>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <control_node/parameters.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/path/sources/reference_path_source.hpp>

namespace control_node {
namespace path {

class ReferencePathManager {
  std::vector<Eigen::Vector2d> m_waypoints;
  std::vector<double> m_data_dist_to_next;
  std::vector<double> m_data_curvature;
  std::vector<double> m_data_target_speed;
  std::vector<double> m_data_track_yaw;

  std::vector<std::pair<int, std::unique_ptr<sources::ReferencePathSource>>> m_sources;
  int m_max_activated_source_idx;

  bool m_changed;

  unsigned int m_dump_paths_uid;
  std::optional<std::filesystem::path> m_dump_paths_dir;

  ReferencePath m_path;

  rclcpp::Logger m_logger;

  const VehicleParameters& m_vp;

  void on_source_notification(int source_id, size_t sz, const sources::ReferencePathSource::UpdateFn& ufn);
public:
  ReferencePathManager(rclcpp::Node* node, const Parameters& p, const VehicleParameters& vp, rclcpp::Logger logger);

  bool changed();
  const ReferencePath& get() const;
};

}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP