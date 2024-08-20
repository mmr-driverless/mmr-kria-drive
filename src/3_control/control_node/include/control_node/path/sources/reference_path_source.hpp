#ifndef CONTROLNODE_PATH_REFERENCEPATHSOURCE
#define CONTROLNODE_PATH_REFERENCEPATHSOURCE

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <control_node/path/reference_path.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace path {
namespace sources {

class ReferencePathSource {
public:
  using WaypointsT = std::span<Eigen::Vector2d>;
  struct UpdateFnResultT {
    bool is_closed;
    ReferencePath::PathData::Metadata data_metadata;

    UpdateFnResultT(bool is_closed, ReferencePath::PathData::Metadata data_metadata)
      : is_closed(is_closed), data_metadata(data_metadata) {}
  };

  // Wrapper to prevent storing the function handle
  class UpdateFn {
    using T = std::function<UpdateFnResultT(WaypointsT, ReferencePath::PathData::Data)>;
  private:
    T m_fn;
  public:
    UpdateFn(T fn) : m_fn(fn) {}
    inline T get() const { return m_fn; }

    UpdateFn() = default;
    UpdateFn(const UpdateFn&) = delete;
    UpdateFn& operator=(const UpdateFn&) = delete;
    UpdateFn(UpdateFn&&) = delete;
    UpdateFn& operator=(UpdateFn&&) = delete;
  };

  using NotificationListener = std::function<void(size_t size, const UpdateFn&)>;

private:
  NotificationListener m_listener;

protected:
  inline void notifyPathChanged(size_t size, const UpdateFn& ufn) const { m_listener(size, ufn); }

  virtual void init_impl(rclcpp::Node& node, const Parameters& params) = 0;

public:
  virtual ~ReferencePathSource() = default;
  inline void init(NotificationListener listener, rclcpp::Node& node, const Parameters& params) { m_listener = listener; init_impl(node, params); }
};

}; // namespace source
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_REFERENCEPATHSOURCE