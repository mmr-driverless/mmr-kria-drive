cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Whether to use the custom visualization msgs or the standard ones
set(USE_KRIA_MSGS ON)
# Whether to use the EDF scheduler or not
set(USE_EDF ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(mmr_base REQUIRED)
if (NOT USE_KRIA_MSGS)
  find_package(visualization_msgs REQUIRED)
endif ()
if (USE_EDF)
  find_package(mmr_edf REQUIRED)
endif ()

add_executable(control_node 
  src/main.cpp
  src/control_node/control_node.cpp
  src/control_node/actuation/canopen_bridge.cpp
  src/control_node/actuation/sim.cpp
  src/control_node/control/pure_pursuit_2023/pure_pursuit_2023.cpp
  src/control_node/control/inspection/inspection.cpp
  src/control_node/estimation/noop_estimator.cpp
  src/control_node/start_stop/start_stop.cpp
)
target_include_directories(control_node PUBLIC include)
set_property(TARGET control_node PROPERTY CXX_STANDARD 20)
target_link_libraries(control_node Eigen3::Eigen)
ament_target_dependencies(control_node rclcpp mmr_base nav_msgs ackermann_msgs)

if (USE_KRIA_MSGS)
  message("Custom visualization messages ENABLED")
  target_compile_definitions(control_node PUBLIC USE_KRIA_MSGS)
else ()
  message("Custom visualization messages DISABLED")
  ament_target_dependencies(control_node visualization_msgs)
endif ()

if (USE_EDF)
  message("EDF scheduling ENABLED")
  find_package(mmr_edf REQUIRED)
  target_compile_definitions(control_node PUBLIC USE_EDF)
  ament_target_dependencies(control_node mmr_edf)
else ()
  message("EDF scheduling DISABLED")
endif ()

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
