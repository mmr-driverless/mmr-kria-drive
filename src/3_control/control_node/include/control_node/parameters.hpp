#ifndef CONTROLNODE_PARAMETERS_HPP
#define CONTROLNODE_PARAMETERS_HPP

#include <exception>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_string_conversions.h>
#include <rmw/time.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace control_node {

class Parameters {
  rclcpp::Node& m_node;
  std::string m_prefix;

  inline std::string get_param_name(const std::string& s) const { return m_prefix + "." + s; }

  template <typename T>
  inline T log_value(const std::string& name, T val) const {
    std::string repr;

    if constexpr (std::is_same_v<T, std::string>)
      repr = val;
    else
      repr = std::to_string(val);

    RCLCPP_DEBUG(m_node.get_logger(), "Parameter %s: %s", name.c_str(), repr.c_str());
    return val;
  }

  inline std::nullopt_t log_missing_value(const std::string& name) const {
    RCLCPP_DEBUG(m_node.get_logger(), "Parameter %s: NOT SET", name.c_str());
    return std::nullopt;
  }

public:
  Parameters(rclcpp::Node* node, const std::string& prefix) : m_node(*node), m_prefix(prefix) {}

  template <typename T>
  T get(const std::string& s, std::optional<T> default_value = std::nullopt) const {
    auto name = get_param_name(s);

    try {
      if (m_node.has_parameter(name))
        return log_value<T>(name, m_node.get_parameter(name).get_value<T>());

      if (default_value.has_value()) {
        std::optional<T> val = get_maybe<T>(s);
        return log_value<T>(name, val.value_or(*default_value));
      }
      else
        return log_value<T>(name, m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type()).get<T>());
    } catch (...) {
      RCLCPP_ERROR(m_node.get_logger(), "Error while parsing parameter '%s'.", name.c_str());
      throw;
    }
  }

  template <typename T>
  std::optional<T> get_maybe(const std::string& s) const {
    auto name = get_param_name(s);

    try {
      if (m_node.has_parameter(name))
        return m_node.get_parameter(name).get_value<T>();

      m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type());

      T ans;
      if (m_node.get_parameter(name, ans))
        return log_value<T>(name, ans);
      else
        return log_missing_value(name);
    } catch (...) {
      RCLCPP_ERROR(m_node.get_logger(), "Error while parsing parameter '%s'.", name.c_str());
      throw;
    }
  }

  rmw_time_t parse_rmw_time(const std::string& prefix, std::optional<rmw_time_t> default_value) const {
    const Parameters p = subparams(prefix);
    rmw_time_t s;
    s.sec = p.get<int>("sec", default_value.has_value()? std::optional<int>(default_value->sec) : std::nullopt);
    s.nsec = p.get<int>("nsec", default_value.has_value()? std::optional<int>(default_value->nsec) : std::nullopt);
    return s;
  }

  rclcpp::QoS parse_qos(const std::string& prefix, rclcpp::QoS default_qos = rclcpp::SystemDefaultsQoS()) const {
    const Parameters p = subparams(prefix);

    auto DEFAULT = default_qos.get_rmw_qos_profile();

    rclcpp::QoS qos = default_qos;
    qos
      .history(rmw_qos_history_policy_from_str(p.get<std::string>("history", rmw_qos_history_policy_to_str(DEFAULT.history)).c_str()))
      .reliability(rmw_qos_reliability_policy_from_str(p.get<std::string>("reliability", rmw_qos_reliability_policy_to_str(DEFAULT.reliability)).c_str()))
      .durability(rmw_qos_durability_policy_from_str(p.get<std::string>("durability", rmw_qos_durability_policy_to_str(DEFAULT.durability)).c_str()))
      .liveliness(rmw_qos_liveliness_policy_from_str(p.get<std::string>("liveliness", rmw_qos_liveliness_policy_to_str(DEFAULT.liveliness)).c_str()))
      .keep_last(p.get<int>("depth", DEFAULT.depth))
      .deadline(p.parse_rmw_time("deadline", DEFAULT.deadline))
      .lifespan(p.parse_rmw_time("lifespan", DEFAULT.lifespan))
      .liveliness_lease_duration(p.parse_rmw_time("liveliness_lease_duration", DEFAULT.liveliness_lease_duration))
      .avoid_ros_namespace_conventions(p.get<bool>("avoid_ros_namespace_conventions", DEFAULT.avoid_ros_namespace_conventions));
    return qos;
  }

  template<typename SentinelFieldT>
  void parse_list(const std::string& prefix, const std::string& sentinel_field, std::function<void(int, const Parameters&)> parser) const {
    const Parameters p = subparams(prefix);
    for (int i = 0; ; ++i) {
      const Parameters p_i = p.subparams(std::to_string(i));

      if (p_i.get_maybe<SentinelFieldT>(sentinel_field))
        parser(i, p_i);
      else
        return;
    }
  }

  Parameters subparams(const std::string& prefix) const {
    return Parameters(&m_node, get_param_name(prefix));
  }
};

}; // namespace control_node

#endif // !CONTROLNODE_PARAMETERS_HPP