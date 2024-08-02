#include <control_node/control_node.hpp>
#include <chrono>

using namespace control_node;

ControlNode::ControlNode() : rclcpp::Node("control_node") {
  m_tick_interval = std::chrono::milliseconds(this->declare_parameter("tick_interval", rclcpp::PARAMETER_INTEGER).get<int>());


}

void ControlNode::tick() {
  RCLCPP_INFO(this->get_logger(), "Tick");
}