#ifndef CONTROLNODE_PARAMETERS_HPP
#define CONTROLNODE_PARAMETERS_HPP

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace control_node {

class Parameters {
  rclcpp::Node& m_node;
  std::string m_prefix;
public:
  Parameters(rclcpp::Node* node, const std::string& prefix) : m_node(*node), m_prefix(prefix) {}

  template <typename T>
  T get(const std::string& s) const {
    auto name = m_prefix + "." + s;

    if (m_node.has_parameter(name))
      return m_node.get_parameter(name).get_value<T>();

    try {
      return m_node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type()).get<T>();
    } catch (...) {
      RCLCPP_ERROR(m_node.get_logger(), "Error while parsing parameter '%s'.", name.c_str());
      throw;
    }
  }
};

}; // namespace control_node

#endif // !CONTROLNODE_PARAMETERS_HPP