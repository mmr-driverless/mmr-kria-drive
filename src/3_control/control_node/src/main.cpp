#include <typeinfo>
#if defined(EDF_MEASURE_EXECUTION_TIME) && defined(USE_EDF)
#include <chrono>
#endif

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

  #ifdef EDF_MEASURE_EXECUTION_TIME
  std::ofstream porcoddio("control_times.csv");
  #endif

  try {
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    auto node = std::make_shared<control_node::ControlNode>();
    executor.add_node(node);

  #ifdef USE_EDF
    while (!stop)
    {
      #ifdef EDF_MEASURE_EXECUTION_TIME
      auto start_t = std::chrono::steady_clock::now();
      #endif

      executor.spin_all(6ms);
      node->tick();

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