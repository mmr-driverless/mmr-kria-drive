cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(mmr_data_logger)
set(NODE_NAME ${PROJECT_NAME}_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mmr_edf REQUIRED)
find_package(mmr_base REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

file(GLOB_RECURSE SOURCES include/*.cpp)
add_executable(
  ${NODE_NAME}
  src/${NODE_NAME}.cpp
  src/${PROJECT_NAME}.cpp
  ${SOURCES}
)


ament_target_dependencies(${NODE_NAME}
  rclcpp
  mmr_edf
  mmr_base
  std_msgs
  geometry_msgs
  sensor_msgs
)


install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
