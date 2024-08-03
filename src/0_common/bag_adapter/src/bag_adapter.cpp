#include "bag_adapter/bag_adapter.hpp"

bag_adapter::bag_adapter(rclcpp::Node::SharedPtr nh, rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr steeringPub){
    this->steeringPub = steeringPub;

    steeringMsg.wheel_angle = 0.0;

    steeringMsg.homing = true;
    steeringMsg.enable = false;
    steeringMsg.disable = false;
    steeringPub->publish(steeringMsg);

    steeringMsg.homing = false;
    steeringMsg.enable = true;
    steeringMsg.disable = false;
    steeringPub->publish(steeringMsg);

    carTargetSub = nh->create_subscription<ackermann_msgs::msg::AckermannDrive>("/sim/drive_parameters", 1, std::bind(&bag_adapter::raceStatusCb, this, std::placeholders::_1));
}

void bag_adapter::raceStatusCb(ackermann_msgs::msg::AckermannDrive::SharedPtr carTargetMsg){
    steeringMsg.homing = false;
    steeringMsg.enable = false;
    steeringMsg.disable = false;

    steeringMsg.wheel_angle = carTargetMsg->steering_angle;

    steeringPub->publish(steeringMsg);
}