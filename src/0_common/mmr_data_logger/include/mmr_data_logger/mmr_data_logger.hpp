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

#include <mmr_edf/mmr_edf.hpp>
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
    std::string statusActuatorTopic, ecuStatusTopic, xsenseTopic, asTopic, missionTopic, lapCounterTopic, conesActualTopic, conesAllTopic, controlTopic;

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
    void ecuCallBack(const mmr_base::msg::EcuStatus::SharedPtr msg);
    
    rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr subActuator;
    void actuatorStatusCallBack(const mmr_base::msg::ActuatorStatus::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subAsState;
    void asStateCallBack(const std_msgs::msg::UInt8::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subMissionSelected;
    void missionSelectedCallBack(const std_msgs::msg::Int8::SharedPtr msg);

    rclcpp::Subscription<mmr_base::msg::RaceStatus>::SharedPtr subRaceStatus;
    void raceStatusCallBack(const mmr_base::msg::RaceStatus::SharedPtr msg);
    
    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesActual;
    void conesActualCallBack(const mmr_base::msg::Marker::SharedPtr msg);

    rclcpp::Subscription<mmr_base::msg::Marker>::SharedPtr subConesAll;
    void conesAllCallBack(const mmr_base::msg::Marker::SharedPtr msg);

    rclcpp::Subscription<mmr_base::msg::ControlLog>::SharedPtr subControl;
    void controlCallBack(const mmr_base::msg::ControlLog::SharedPtr msg);

    void load_parameters();


    public:

    MMR_Data_Logger();

};