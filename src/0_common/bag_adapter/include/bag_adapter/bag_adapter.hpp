#include <mmr_kria_base/msg/cmd_motor.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <rclcpp/rclcpp.hpp>


class bag_adapter{
    public:
    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr carTargetSub;
    bag_adapter(rclcpp::Node::SharedPtr nh, rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr steeringPub);
    bag_adapter(){};
    void raceStatusCb(ackermann_msgs::msg::AckermannDrive::SharedPtr carTargetMsg);
    rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr steeringPub;
    mmr_kria_base::msg::CmdMotor steeringMsg;
};
