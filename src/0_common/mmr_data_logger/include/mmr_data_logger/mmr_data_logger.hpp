/*
TODO
-aggiungere la speed quando simo ha finito di creare il topic e il messaggio 
-scegliere se usare il messaggio marker o array_marker [ DONE ]
*/

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iostream>

#include <mmr_edf/mmr_edf.hpp>
#include <rclcpp/qos.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/msg/cmd_motor.hpp>
#include <mmr_base/msg/control_log.hpp>
#include <mmr_base/msg/race_status.hpp>
#include <mmr_base/msg/marker.hpp>
#include <mmr_base/msg/marker_array.hpp>

#include <string.h>

class MMR_Data_Logger : public EDFNode
{
private:

    const float MAXIMUM_PBRAKE = 10;
    const float STEERING_ANGLE_SCALE_FACTOR= 0.5;

    std::string statusActuatorTopic, ecuStatusTopic, xsenseTopic, asTopic, missionTopic, lapCounterTopic, conesActualTopic, conesAllTopic, controlTopic;
    float pbrake_rear, pbrake_front, pebs1, pebs2;
    bool debug;

    /* DV driving dynamics 1 */ 

    uint8_t speedTarget, speedActual, brakeActual, brakeTarget;
    int8_t steeringAgleActual,steeringAngleTarget;

    /* DV driving dynamics 2 */

    int16_t accelerationLongitudinal, accelerationLateral, yawRate;

    /* DV system status */

    int8_t asStatus, ebsState, missionSelected, serviceBrakeState;
    bool steeringState;
    int8_t lapCounter, conesCountActual, conesCountAll;

    rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr subEcu;
    void ecuCallBack(const mmr_base::msg::EcuStatus::SharedPtr msg){
        this->pbrake_front=msg->p_brake_front;
        this->pbrake_rear=msg->p_brake_rear;
        this->pebs1=msg->p_ebs_1;
        this->pebs2=msg->p_ebs_2;
        this->speedActual=static_cast<uint8_t>(msg->vehicle_speed);
        this->brakeActual=static_cast<uint8_t>((((pbrake_front+pbrake_rear)/2)/MAXIMUM_PBRAKE)*100);
        this->steeringAgleActual=static_cast<uint8_t>(msg->steering_angle/STEERING_ANGLE_SCALE_FACTOR);
        if( this->debug ){
        std::cout<<"ECU msg: "<<std::endl;
        std::cout<<"VEHICLE SEED: "<<msg->vehicle_speed<<std::endl;
        std::cout<<"PBRAKE_FRONT: "<<pbrake_front<<std::endl;
        std::cout<<"PBRAKE REAR: "<<pbrake_rear<<std::endl;
        std::cout<<"PEBS1: "<<pebs1<<std::endl;
        std::cout<<"PEBS2: "<<pebs2<<std::endl;
        std::cout<<"STEERING ANGLE: "<<msg->steering_angle<<std::endl;
        std::cout<<"ELABORATED INFORMATION msg: "<<std::endl;
        std::cout<<"SPEED ACTUAL: "<<static_cast<int>(speedActual)<<std::endl;
        std::cout<<"BRAKE ACTUAL: "<<static_cast<int>(brakeActual)<<std::endl;
        std::cout<<"STEERING ACTUAL: "<<static_cast<int>(steeringAgleActual)<<std::endl;
        std::cout<<std::endl;
        }
    }
    
    rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr subActuator;
    void actuatorStatusCallBack(const mmr_base::msg::ActuatorStatus::SharedPtr msg){}

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg){}

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subAsState;
    void asStateCallBack(const std_msgs::msg::UInt8::SharedPtr msg){}

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subMissionSelected;
    void missionSelectedCallBack(const std_msgs::msg::Int8::SharedPtr msg){}

    rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr subRaceStatus;
    void raceStatusCallBack(const mmr_base::msg::RaceStatus::SharedPtr msg){}
    
    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesActual;
    void conesActualCallBack(const mmr_base::msg::Marker::SharedPtr msg){}

    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesAll;
    void conesAllCallBack(const mmr_base::msg::Marker::SharedPtr msg){}

    rclcpp::Subscription<mmr_base::msg::ControlLog>::SharedPtr subControl;
    void controlCallBack(const mmr_base::msg::ControlLog::SharedPtr msg){}


public:

    MMR_Data_Logger();
    void load_parameters();
    ~MMR_Data_Logger();

};