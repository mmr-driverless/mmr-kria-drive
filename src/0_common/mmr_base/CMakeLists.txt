cmake_minimum_required(VERSION 3.8)
project(mmr_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdMotor.msg"
  "msg/ActuatorStatus.msg"
  "msg/EcuStatus.msg"
  "msg/ResStatus.msg"
  "msg/UVCoordinate.msg"
  "msg/MeshFile.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "msg/CmdEcu.msg"
  "msg/ControlLog.msg"
  "msg/PurePursuitLog.msg"
  "msg/RaceStatus.msg"
  ### IMU data dependecies ###
  "msg/Imu/Acceleration.msg"
  "msg/Imu/EulerAngles.msg"
  "msg/Imu/Gnss.msg"
  "msg/Imu/GnssStatus.msg"
  "msg/Imu/Quaternion.msg"
  "msg/Imu/RateOfTurn.msg"
  "msg/Imu/UTCTime.msg"
  "msg/Imu/Velocity.msg"
  ############################
  "msg/ImuCanData.msg"
  "msg/SpeedProfilePoint.msg"
  "msg/SpeedProfilePoints.msg"
  # Add custom msg dependecies to standard ROS2 msgs
  DEPENDENCIES 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  ackermann_msgs
)

install(FILES
  config/configuration.hpp
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

ament_package()