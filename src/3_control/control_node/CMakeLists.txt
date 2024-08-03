cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(USE_KRIA_MSGS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(mmr_base REQUIRED)

add_executable(control_node 
  src/main.cpp
  src/control_node/control_node.cpp
  src/control_node/actuation/canopen_bridge.cpp
  src/control_node/control/pure_pursuit_2023/pure_pursuit_2023.cpp
  src/control_node/estimation/noop_estimator.cpp
)
target_include_directories(control_node PUBLIC include)
set_property(TARGET control_node PROPERTY CXX_STANDARD 20)
target_link_libraries(control_node Eigen3::Eigen)
ament_target_dependencies(control_node rclcpp mmr_base nav_msgs ackermann_msgs)

if (USE_KRIA_MSGS)
  target_compile_definitions(control_node PUBLIC USE_KRIA_MSGS)
else ()
  find_package(visualization_msgs REQUIRED)
  ament_target_dependencies(control_node visualization_msgs)
endif ()

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
