#include <control_node/control_node.hpp>
#include <chrono>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<control_node::ControlNode>();
  auto timer = node->create_wall_timer(
    node->tick_interval(),
    [&node]() { node->tick(); }
  );
  rclcpp::spin(node);

  rclcpp::shutdown();
}