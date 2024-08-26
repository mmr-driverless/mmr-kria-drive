# Standard ROS 2 message header
std_msgs/Header header

# steer status -> follow the configuration.hpp
uint8 steer_status

# brake status -> follow the configuration.hpp
uint8 brake_status

# clutch status -> follow the configuration.hpp
uint8 clutch_status

# voltage value in 0.1V
float32 voltage