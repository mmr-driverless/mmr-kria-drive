#ifndef CONTROLNODE_COMPONENTFACTORY_HPP
#define CONTROLNODE_COMPONENTFACTORY_HPP

#include <utility>
#include <memory>
#include <vector>

#include <control_node/parameters.hpp>

namespace control_node {

template <typename T>
class ComponentFactory {
  std::initializer_list<std::pair<const char*, std::unique_ptr<T>(*)()>> m_factories;

public:
  constexpr ComponentFactory(std::initializer_list<std::pair<const char*, std::unique_ptr<T>(*)()>> factories)
    : m_factories(factories)
  { }

  std::unique_ptr<T> get(const std::string& type) const {
    // Find the factory of the given type.
    for (auto p : m_factories)
      if (type == p.first)
        return p.second();
    return nullptr;
  }

  /**
    @brief Create a component for each entry in the parameter list.
    @param p The parent of the component list (the list is resolved as parent->prefix)
    @param prefix The name of the component list (the list is resolved as parent->prefix)
    @param init_fn The initialization function which is called for each component. It receives a reference to the new component, the index, a Parameters object for that component, and its type name.
    @param logger A logger.
   */
  std::vector<std::pair<int, std::unique_ptr<T>>> from_param_list(const Parameters& p, const std::string& prefix, std::function<void(T&, int, const Parameters&, const std::string&)> init_fn, const rclcpp::Logger& logger) const {
    std::vector<std::pair<int, std::unique_ptr<T>>> ans;

    // For each entry in the list - we expect each entry to have the "type" field, which we use as sentinel
    p.parse_list<std::string>(prefix, "type", [this, &ans, &init_fn, &logger](int idx, const Parameters& p_i) {
      // If the entry is disabled
      if (!p_i.get<bool>("enabled")) {
        RCLCPP_WARN(logger, "Entry %d IGNORED (disabled from config).", idx);
        return;
      }

      // Build the component
      std::string type = p_i.get<std::string>("type");
      std::unique_ptr<T> component = this->get(type);
      if (component == nullptr) {
        RCLCPP_ERROR(logger, "Entry %d IGNORED (UNKNOWN type '%s')", idx, type.c_str());
        return;
      }

      // Store the component in the result
      ans.push_back(std::make_pair(idx, std::move(component)));

      // Call the initialization function for this component
      RCLCPP_INFO(logger, "INITIALIZING entry %d (of type '%s')", idx, type.c_str());
      init_fn(*ans.back().second, idx, p_i.subparams("params"), type);
    });

    return ans;
  }
};

};


#endif // !CONTROLNODE_COMPONENTFACTORY_HPP