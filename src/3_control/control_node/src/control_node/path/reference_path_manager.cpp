#include <algorithm>
#include <control_node/path/sources/reference_path_source.hpp>
#include <control_node/path/reference_path_manager.hpp>
#include <control_node/path/sources/sources_factory.hpp>
#include <vector>

namespace control_node {
namespace path {

ReferencePathManager::ReferencePathManager(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) : m_logger(logger) {
  m_sources = sources::get_factory().from_param_list(p, "sources", [this, &node](sources::ReferencePathSource& src, int idx, const Parameters& p_i) {
    src.init(
      std::bind(&ReferencePathManager::on_source_notification, this, idx, std::placeholders::_1, std::placeholders::_2),
      node,
      p_i
    );
  }, m_logger.get_child("ComponentFactory"));

  RCLCPP_INFO(m_logger, "Initialization COMPLETED (%zu sources).", m_sources.size());
}

void ReferencePathManager::on_source_notification(int source_id, size_t path_size, const sources::ReferencePathSource::UpdateFn& ufn) {
  RCLCPP_INFO(m_logger, "RECEIVED path from source %d.", source_id);

  // Delete any source that has id > source_id.
  auto it = std::find_if(m_sources.begin(), m_sources.end(), [source_id](std::pair<int, std::unique_ptr<sources::ReferencePathSource>>& src) {
    return src.first > source_id;
  });
  m_sources.erase(it, m_sources.end());

  m_changed = true;

  // Accomodate for the path size
  m_waypoints.resize(path_size);
  m_data.resize(path_size);

  auto waypoints_view = std::span<decltype(m_waypoints)::value_type>(m_waypoints);
  auto data_view = std::span<decltype(m_data)::value_type>(m_data);

  // Update the path
  auto props = ufn.get()(waypoints_view, data_view);
  m_path = ReferencePath(waypoints_view, data_view, props);
  m_path.compute_data();
}


bool ReferencePathManager::changed() {
  bool ans = m_changed;
  m_changed = false;
  return ans;
}
const ReferencePath& ReferencePathManager::get() const { return m_path; }

}; // namespace path
}; // namespace control_node