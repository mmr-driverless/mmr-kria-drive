
#ifdef EDF_MEASURE_EXECUTION_TIME
#include <fstream>
#include <chrono>
#endif

#include <canopen_bridge/canopen_bridge.hpp>


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
    std::ofstream porcoddio("canopen_times.csv");
    #endif

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<CANOpenBridge>();
    executor.add_node(node);
    
    while (true)
    {
      #ifdef EDF_MEASURE_EXECUTION_TIME
      auto start_t = std::chrono::steady_clock::now();
      #endif

      executor.spin_all(4ms);
      node->monitorSteer();
      node->sendActuatorStatus();

      #ifdef LOG_POWER_CONSUPTION_ACT
      node->logMaxonPower();
      #endif

      #ifdef EDF_MEASURE_EXECUTION_TIME
      auto end_t = std::chrono::steady_clock::now();
      unsigned int cpu_number;
      getcpu(&cpu_number, NULL);
      porcoddio
      << std::chrono::duration_cast<std::chrono::nanoseconds>(start_t.time_since_epoch()).count()
      << "," << std::chrono::duration_cast<std::chrono::nanoseconds>(end_t - start_t).count()
      << "," << cpu_number
      << "\n";
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



