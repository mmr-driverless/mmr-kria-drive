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

  std::vector<std::pair<int, std::unique_ptr<T>>> from_param_list(const Parameters& p, const std::string& prefix, std::function<void(T&, int, const Parameters&)> init_fn, const rclcpp::Logger& logger) const {
    std::vector<std::pair<int, std::unique_ptr<T>>> ans;

    p.parse_list<std::string>(prefix, "type", [this, &ans, &init_fn, &logger](int idx, const Parameters& p_i) {
      if (!p_i.get<bool>("enabled")) {
        RCLCPP_WARN(logger, "Entry %d IGNORED (disabled from config).", idx);
        return;
      }

      std::string type = p_i.get<std::string>("type");
      auto component = this->get(type);
      if (component == nullptr) {
        RCLCPP_ERROR(logger, "Entry %d IGNORED (UNKNOWN type '%s')", idx, type.c_str());
        return;
      }

      ans.push_back(std::make_pair(idx, std::move(component)));
      RCLCPP_INFO(logger, "INITIALIZING entry %d (of type '%s')", idx, type.c_str());
      init_fn(*ans.back().second, idx, p_i.subparams("params"));
    });

    return ans;
  }
};

};


#endif // !CONTROLNODE_COMPONENTFACTORY_HPP