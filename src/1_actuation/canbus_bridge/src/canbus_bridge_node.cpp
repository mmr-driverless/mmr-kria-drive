#ifdef EDF_MEASURE_EXECUTION_TIME
#include <fstream>
#include <chrono>
#endif

#include <canbus_bridge/canbus_bridge.hpp>

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
  /* node initialization */
  rclcpp::init(argc, argv);

  try
  {
    #ifdef EDF_MEASURE_EXECUTION_TIME
    std::ofstream porcoddio("canbus_times.txt");
    #endif

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<CANBusBridge>();
    executor.add_node(node);
    
    while (true)
    {
      #ifdef EDF_MEASURE_EXECUTION_TIME
      auto start_t = std::chrono::steady_clock::now();
      #endif

      executor.spin_all(10s);
      node->readMsgFromCANBus();
      node->sendStatus();
      node->changeGearUpDown();
      node->setLaunchControl();
      node->setGearNeutral();
      node->send24VCockpit();

      #ifdef EDF_MEASURE_EXECUTION_TIME
      auto end_t = std::chrono::steady_clock::now();
      porcoddio << std::chrono::duration_cast<std::chrono::nanoseconds>(end_t - start_t).count() << "\n";
      #endif
      
      sched_yield();
    }

    rclcpp::shutdown();
  }
  catch(const rclcpp::exceptions::InvalidNodeError& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
