
#include "bag_adapter/bag_adapter.hpp"

rclcpp::Publisher<mmr_kria_base::msg::CmdMotor>::SharedPtr steeringPub;

void disableMotor(){
    mmr_kria_base::msg::CmdMotor steeringMsg;

    steeringMsg.homing = false;
    steeringMsg.enable = false;
    steeringMsg.disable = true;
    steeringMsg.wheel_angle = 0.0;

    steeringPub->publish(steeringMsg);
}

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing node process.\n";
        disableMotor();
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv){
    signal(SIGINT, handleSignal);
    rclcpp::init(argc, argv );
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("bag_adapter_node");

    steeringPub = nh->create_publisher<mmr_kria_base::msg::CmdMotor>("/command/steer", 1);
    bag_adapter adapter(nh, steeringPub);

    rclcpp::spin(nh);

    return 0;
}