#include <control_node/control_node.hpp>

bool stop = false;
void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Received SIGINT. Requesting stop." << std::endl;
    stop = true;
  }
}

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<control_node::ControlNode>();
  executor.add_node(node);

  while (!stop)
  {
    executor.spin_all(std::chrono::duration_cast<std::chrono::nanoseconds>(node->tick_interval()) / 2);
    node->tick();
    sched_yield();
  }

  rclcpp::shutdown();
  return 0;
}