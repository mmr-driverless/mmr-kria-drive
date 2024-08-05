#include <control_node/path/sources/reference_path_source.hpp>
#include <control_node/path/reference_path_manager.hpp>
#include <control_node/path/sources/sources_factory.hpp>

namespace control_node {
namespace path {

ReferencePathManager::ReferencePathManager(rclcpp::Node& node, const Parameters& p, const rclcpp::Logger& logger) : m_max_activated_source_idx(0), m_logger(logger) {
  // Parse the list, by using the field "type" as sentinel
  p.parse_list<std::string>("sources", "type", [&node, this](int i, const Parameters& params) {
    // This lambda is called for each list entry.


    // If this source is not enabled, ignore it.
    bool enabled = params.get<bool>("enabled");
    if (!enabled) {
      RCLCPP_WARN(m_logger, "Source %d IGNORED (disabled from config).", i);
      return;
    }

    // Make the appropriate path source from the type.
    std::string type = params.get<std::string>("type");
    auto src = sources::make_path_source(type);
    if (src == nullptr) {
      RCLCPP_ERROR(m_logger, "Source %d IGNORED (UNKNOWN path source type '%s')", i, type.c_str());
      return;
    }

    // Initialize the source
    m_sources.push_back(std::move(src));
    m_sources.back()->init(
      std::bind(&ReferencePathManager::on_source_notification, this, i, std::placeholders::_1, std::placeholders::_2),
      node,
      params.subparams("params")
    );
  });

  RCLCPP_INFO(m_logger, "Initialization COMPLETED (%zu sources).", m_sources.size());
}

void ReferencePathManager::on_source_notification(int source_id, size_t path_size, const sources::ReferencePathSource::UpdateFn& ufn) {
  // Delete any source that has id < source_id.
  for (int i = m_max_activated_source_idx; i < source_id; ++i)
    m_sources[i].reset();

  RCLCPP_INFO(m_logger, "RECEIVED path from source %d.", source_id);

  m_max_activated_source_idx = source_id;
  if (source_id >= m_max_activated_source_idx) {
    m_max_activated_source_idx = source_id;
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
}


bool ReferencePathManager::changed() {
  bool ans = m_changed;
  m_changed = false;
  return ans;
}
const ReferencePath& ReferencePathManager::get() const { return m_path; }

}; // namespace path
}; // namespace control_node