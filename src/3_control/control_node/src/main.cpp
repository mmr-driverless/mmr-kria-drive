#include <typeinfo>

#include <control_node/control_node.hpp>

#ifdef USE_EDF
bool stop = false;
void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Received SIGINT. Requesting stop." << std::endl;
    stop = true;
  }
}
using namespace std::chrono_literals;
#endif

int main(int argc, char * argv[])
{
#ifdef USE_EDF
  signal(SIGINT, handleSignal);
#endif

  /* node initialization */
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<control_node::ControlNode>();
    executor.add_node(node);

  #ifdef USE_EDF
    while (!stop)
    {
      executor.spin_all(std::chrono::duration_cast<std::chrono::nanoseconds>(node->tick_interval()) / 2);
      node->tick();
      sched_yield();
    }
  #else
    auto timer = node->create_wall_timer(node->tick_interval(), std::bind(&control_node::ControlNode::tick, node.get()));
    executor.spin();
  #endif

  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Terminating due to exception of type '%s' - what(): %s", typeid(e).name(), e.what());
    rclcpp::shutdown();
    throw;
  }

  return 0;
}