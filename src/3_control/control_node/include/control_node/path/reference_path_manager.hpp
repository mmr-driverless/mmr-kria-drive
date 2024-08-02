#ifndef CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP
#define CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <control_node/parameters.hpp>

namespace control_node {

class ReferencePathManager {
public:
  ReferencePathManager(rclcpp::Node node, const Parameters& p);

  const ReferencePath& get();
};

}; // namespace control_node


#endif // !CONTROLNODE_PATH_REFERENCEPATHMANAGER_HPP