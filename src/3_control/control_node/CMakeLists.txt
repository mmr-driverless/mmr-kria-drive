cmake_minimum_required(VERSION 3.8)
project(control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(control_node src/demo.cpp)
target_include_directories(control_node PUBLIC include)
set_property(TARGET control_node PROPERTY CXX_STANDARD 20)
target_link_libraries(control_node Eigen3::Eigen)
ament_target_dependencies(control_node rclcpp visualization_msgs nav_msgs ackermann_msgs)

install(
  TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
