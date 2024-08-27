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
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
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
#include <mmr_base/msg/pure_pursuit_log.hpp>
#include <mmr_base/configuration.hpp>
#include <can_msgs/msg/frame.hpp>

#include <string.h>

class MMR_Data_Logger : public EDFNode
{
private:

    const float MAXIMUM_PBRAKE = 10;
    const float STEERING_ANGLE_SCALE_FACTOR= 2;
    const float ACCELERATION_SCALE_FACTOR= 512;
    const float YAW_SCALE_FACTOR= 128;
    const float MAXIMUM_STEERING_ANGLE = 120.0;

    bool debug,brakeMotorEnabled;
    std::string purePursuitTopic, sendMsgTopic, statusActuatorTopic, ecuStatusTopic, xsenseTopic, asTopic, missionTopic, lapCounterTopic, conesActualTopic, conesAllTopic, controlTopic;
    float pbrake_rear, pbrake_front, pebs1, pebs2, steerWheelRate;
    

    /* DV driving dynamics 1 */ 

    uint8_t speedTarget, speedActual, brakeActual, brakeTarget;
    int8_t steeringAgleActual,steeringAngleTarget;

    /* DV driving dynamics 2 */

    int16_t accelerationLongitudinal, accelerationLateral, yawRate;

    /* DV system status */

    int8_t asStatus, ebsState, missionSelected, serviceBrakeState;
    bool steeringState;
    int8_t lapCounter, conesCountActual;
    int16_t conesCountAll;

    rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr subEcu;
    void ecuCallBack(const mmr_base::msg::EcuStatus::SharedPtr msg){
        this->pbrake_front=msg->p_brake_front;
        this->pbrake_rear=msg->p_brake_rear;
        this->pebs1=msg->p_ebs_1;
        this->pebs2=msg->p_ebs_2;
        this->speedActual=static_cast<uint8_t>(msg->vehicle_speed);
        this->brakeActual=static_cast<uint8_t>((((pbrake_front+pbrake_rear)/2)/MAXIMUM_PBRAKE)*100);
        float precentageOfSteering = ((msg->steering_angle/this->steerWheelRate )*STEERING_ANGLE_SCALE_FACTOR);
        this->steeringAgleActual=static_cast<int8_t>(precentageOfSteering);

        if(brakeMotorEnabled and this->pbrake_front>0 and this->pbrake_rear>0){
            this->serviceBrakeState=2;
        }else if(brakeMotorEnabled and this->pbrake_front<=0 and this->pbrake_rear<=0){
            this->serviceBrakeState=1;
        }

        if(pebs1<=0 and pebs2<=0){
            this->ebsState=1;
        }else if (pebs1>0 and pebs2>0 and pbrake_front>=20 and pbrake_rear>=20){
            this->ebsState=3;            
        }else if ( pebs1>0 and pebs2>0 ){
            this->ebsState=2;
        }
    }
    
    rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr subActuator;
    void actuatorStatusCallBack(const mmr_base::msg::ActuatorStatus::SharedPtr msg){
        if(msg->steer_status>0){
            this->steeringState=true;
        }
        if(msg->brake_status==0){
            this->serviceBrakeState=3;
        }else if( msg->brake_status>0){
            this->brakeMotorEnabled=true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg){
        this->accelerationLongitudinal=static_cast<int16_t>(msg->linear_acceleration.x*ACCELERATION_SCALE_FACTOR);
        this->accelerationLateral=static_cast<int16_t>(msg->linear_acceleration.y*ACCELERATION_SCALE_FACTOR);
        this->yawRate=static_cast<int16_t>(msg->angular_velocity.z*YAW_SCALE_FACTOR);

        if(this->debug){
            std::cout<<"-------------IMU MSG---------------------------"<<std::endl;
            std::cout<<"ACCELERATION LONGITUDINAL: "<<msg->linear_acceleration.x<<std::endl;
            std::cout<<"ACCELERATION LATERAL: "<<msg->linear_acceleration.y<<std::endl;
            std::cout<<"ACCELERATION Z: "<<msg->linear_acceleration.z<<std::endl;
            std::cout<<"YAW RATE: "<<msg->angular_velocity.z<<std::endl;
            std::cout<<"--------ELABORATED INFORMATION msg-------------"<<std::endl;
            std::cout<<"ACCELERATION LONGITUDINAL: "<<this->accelerationLongitudinal<<std::endl;
            std::cout<<"ACCELERATION LATERAL: "<<this->accelerationLateral<<std::endl;
            std::cout<<"YAW RATE: "<<this->yawRate<<std::endl;
            std::cout<<"-----------------------------------------------"<<std::endl;
            std::cout<<std::endl;
        }

    }

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subAsState;
    void asStateCallBack(const std_msgs::msg::Int8::SharedPtr msg){
        switch(msg->data){
            case AS::STATE::OFF: this->asStatus=1;break;
            case AS::STATE::READY: this->asStatus=2;break;
            case AS::STATE::DRIVING: this->asStatus=3;break;
            case AS::STATE::EMERGENCY: this->asStatus=4;break;
            case AS::STATE::FINISHED: this->asStatus=5;break;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subMissionSelected;
    void missionSelectedCallBack(const std_msgs::msg::Int8::SharedPtr msg){
        switch(msg->data){
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_ACCELERATION: this->missionSelected=1;break;
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_SKIDPAD: this->missionSelected=2;break;
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_TRACKDRIVE: this->missionSelected=3;break;
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_EBS_TEST: this->missionSelected=4;break;
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_INSPECTION: this->missionSelected=5;break;
            case COCKPIT::MMR_MISSION_VALUE::MMR_MISSION_AUTOCROSS: this->missionSelected=6;break;
        }
    }

    rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr subRaceStatus;
    void raceStatusCallBack(const mmr_base::msg::RaceStatus::SharedPtr msg){
        this->lapCounter=msg->current_lap;
    }
    
    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesActual;
    void conesActualCallBack(const mmr_base::msg::Marker::SharedPtr msg){
        const std::vector<geometry_msgs::msg::Point>& points = msg->points;
        this->conesCountActual = points.size();
    }

    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesAll;
    void conesAllCallBack(const mmr_base::msg::Marker::SharedPtr msg){
        const std::vector<geometry_msgs::msg::Point>& points = msg->points;
        this->conesCountAll = points.size();
    }

    rclcpp::Subscription<mmr_base::msg::ControlLog>::SharedPtr subControl;
    void controlCallBack(const mmr_base::msg::ControlLog::SharedPtr msg){
        float percentageSteeringAngle=(((msg->steer/this->steerWheelRate ) )/STEERING_ANGLE_SCALE_FACTOR);
        this->steeringAngleTarget=static_cast<int8_t>(percentageSteeringAngle);
        this->brakeTarget=static_cast<int8_t>((msg->brake/MAXIMUM_PBRAKE)*100);
    }

    rclcpp::Subscription<mmr_base::msg::PurePursuitLog>::SharedPtr subPurePursuitLog;
    void pplCallBack(const mmr_base::msg::ControlLog::SharedPtr msg){
        this->speedTarget=static_cast<uint8_t>(msg->smoothed_target_speed_m_s*3,6);
        this->speedActual=static_cast<uint8_t>(msg->current_speed_m_s*3,6);
    }

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pubMsg;


    uint64_t pack_bits(uint64_t value, int position, int length);
    uint64_t create_dv_driving_dynamics_1_message();
    uint64_t create_dv_driving_dynamics_2_message();
    uint64_t create_dv_system_status_messagge();




public:

    MMR_Data_Logger();
    void load_parameters();
    ~MMR_Data_Logger();
    void send_messages();
    void print_parameters();

};