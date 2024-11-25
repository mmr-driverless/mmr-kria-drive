cmake_minimum_required(VERSION 3.8)
project(canopen_bridge)
set (CMAKE_CXX_STANDARD 20)

# Whether to log the execution times to a file
set(EDF_MEASURE_EXECUTION_TIME OFF)
set(LOG_POWER_CONSUPTION_ACT ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(mmr_edf REQUIRED)

include_directories(
  include/
)

add_library(motor_maxon src/driver/motor_maxon.cpp)
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

ament_target_dependencies(motor_maxon
  mmr_base
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  can_msgs
  mmr_base
  mmr_edf
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

if (EDF_MEASURE_EXECUTION_TIME)
  message("EDF Execution time logging ENABLED")
  target_compile_definitions(${PROJECT_NAME}_node PUBLIC EDF_MEASURE_EXECUTION_TIME)
else()
  message("EDF Execution time logging DISABLED")
endif ()

if (LOG_POWER_CONSUPTION_ACT)
  message("Check power consuption of Maxon motor")
  target_compile_definitions(${PROJECT_NAME}_node PUBLIC LOG_POWER_CONSUPTION_ACT)
else()
  message("not read power consuption")
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  motor_maxon
)

install(TARGETS
  ${PROJECT_NAME}_node
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
