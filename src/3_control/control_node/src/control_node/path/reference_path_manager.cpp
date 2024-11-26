#include <stdexcept>
#include <vector>

#include <control_node/path/sources/reference_path_source.hpp>
#include <control_node/path/sources/sources_factory.hpp>

#include <control_node/path/reference_path_manager.hpp>

namespace control_node {
namespace path {

ReferencePathManager::ReferencePathManager(rclcpp::Node* node, const Parameters& p, const VehicleParameters& vp, rclcpp::Logger logger)
  : m_max_activated_source_idx(0),
    m_changed(false),
    m_dump_paths_uid(0),
    m_logger(logger),
    m_vp(vp)
{

  m_sources = sources::get_factory().from_param_list(p, "sources", [this, &node](sources::ReferencePathSource& src, int idx, const Parameters& p_i, const std::string&) {
    src.init(
      std::bind(&ReferencePathManager::on_source_notification, this, idx, std::placeholders::_1, std::placeholders::_2),
      *node,
      p_i
    );
  }, m_logger.get_child("ComponentFactory"));

  auto dump_dir = p.get_maybe<std::string>("dump_paths_dir");
  if (dump_dir) {
    m_dump_paths_dir = std::filesystem::path(*dump_dir);
    RCLCPP_WARN(m_logger, "PATH DUMPING IS ENABLED INTO '%s'", m_dump_paths_dir->c_str());

    if (std::filesystem::exists(*m_dump_paths_dir)) {
      RCLCPP_FATAL(m_logger, "The dump directory already exists! Not overwriting.");
      throw std::invalid_argument("dump_paths_dir");
    }

    if (!std::filesystem::create_directory(*m_dump_paths_dir)) {
      RCLCPP_FATAL(m_logger, "Failed to create the dump directory.");
      throw std::invalid_argument("dump_paths_dir");
    }
  }

  RCLCPP_INFO(m_logger, "Initialization COMPLETED (%zu sources).", m_sources.size());
}

void ReferencePathManager::on_source_notification(int source_id, size_t path_size, const sources::ReferencePathSource::UpdateFn& ufn) {
  if (source_id < m_max_activated_source_idx) {
    RCLCPP_INFO(m_logger, "IGNORING path received from source %d.", source_id);
    return;
  }
  
  RCLCPP_INFO(m_logger, "RECEIVED path from source %d (%zu waypoints)", source_id, path_size);
  if (source_id >= m_max_activated_source_idx)
    m_max_activated_source_idx = source_id;

  m_changed = true;

  // Accomodate for the path size
  m_waypoints.resize(path_size);
  m_data_curvature.resize(path_size);
  m_data_dist_to_next.resize(path_size);
  m_data_target_speed.resize(path_size);

  // Update the path
  auto data = ReferencePath::PathData(m_data_dist_to_next, m_data_curvature, m_data_target_speed);
  auto result = ufn.get()(
    m_waypoints,
    data.data
  );
  data.metadata = result.data_metadata;
  m_path = ReferencePath(m_waypoints, data, result.is_closed);
  m_path.compute_data(m_vp);

  if (m_dump_paths_dir.has_value()) {
    m_path.dump(*m_dump_paths_dir / (std::to_string(m_dump_paths_uid) + "_src" + std::to_string(source_id) + ".csv"));
    ++m_dump_paths_uid;
  }
}


bool ReferencePathManager::changed() {
  bool ans = m_changed;
  m_changed = false;
  return ans;
}
const ReferencePath& ReferencePathManager::get() const { return m_path; }

}; // namespace path
}; // namespace control_node