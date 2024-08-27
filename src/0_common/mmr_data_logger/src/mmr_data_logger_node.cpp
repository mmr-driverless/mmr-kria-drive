#include <mmr_data_logger/mmr_data_logger.hpp>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing node process.\n";
        rclcpp::shutdown();
    }
}

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  signal(SIGINT, handleSignal);
    signal(SIGINT, handleSignal);
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    decltype(auto) node = std::make_shared<MMR_Data_Logger>();
    executor.add_node(node);

    while (true) {
      executor.spin_all(30ms);
      node->send_messages();
      sched_yield();
    }

    rclcpp::shutdown();
  }
  catch (const rclcpp::exceptions::InvalidNodeError &e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}
