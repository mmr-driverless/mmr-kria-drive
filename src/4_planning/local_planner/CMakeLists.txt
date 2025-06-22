cmake_minimum_required(VERSION 3.5)
project(local_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Whether to use the EDF scheduler or not
set(USE_EDF OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
if (USE_EDF)
  find_package(mmr_edf REQUIRED)
endif ()

# Include directories
include_directories(
  include/local_planner
  ${EIGEN3_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}   # use lowercase variable from tf2_eigen
)

#---------------------------------------------------------
#                            ACCELERATION
#---------------------------------------------------------

add_library(acceleration src/acceleration.cpp)

ament_target_dependencies(acceleration
  rclcpp
  geometry_msgs
  nav_msgs
  mmr_base
)

#---------------------------------------------------------
#                             AUTOCROSS
#---------------------------------------------------------

add_library(autocross
  src/autocross.cpp
  src/modules/DelaunayTri.cpp
  src/modules/Visualization.cpp
  src/modules/WayComputer.cpp
  src/structures/Circle.cpp
  src/structures/Edge.cpp
  src/structures/Node.cpp
  src/structures/Point.cpp
  src/structures/Trace.cpp
  src/structures/Triangle.cpp
  src/structures/Vector.cpp
  src/structures/Way.cpp
  src/utils/Failsafe.cpp
  src/utils/KDTree.cpp
  src/utils/Params.cpp
  src/utils/Time.cpp
)

ament_target_dependencies(autocross
  rclcpp
  geometry_msgs
  nav_msgs
  mmr_base
)

target_link_libraries(autocross
  Eigen3::Eigen
  ${tf2_eigen_LIBRARIES}   # use lowercase variable for linking
)

#---------------------------------------------------------
#                             SKIDPAD
#---------------------------------------------------------

add_library(skidpad src/skidpad.cpp)

ament_target_dependencies(skidpad
  rclcpp
  geometry_msgs
  mmr_base
  nav_msgs
)

#---------------------------------------------------------
#                             NODE
#---------------------------------------------------------

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  acceleration
  autocross
  skidpad
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  nav_msgs
  mmr_base
  rmw
)

#_________________________________________________________
#                             DIRECTORIES
#_________________________________________________________

if (USE_EDF)
  ament_target_dependencies(acceleration mmr_edf)
  ament_target_dependencies(autocross mmr_edf)
  ament_target_dependencies(skidpad mmr_edf)
  ament_target_dependencies(${PROJECT_NAME}_node mmr_edf)
endif()

install(TARGETS
  acceleration
  autocross
  skidpad
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
