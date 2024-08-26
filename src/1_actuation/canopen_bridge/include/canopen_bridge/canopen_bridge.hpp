#pragma once

#include <canopen_bridge/driver/steer_maxon.hpp>
#include <canopen_bridge/driver/brake_maxon.hpp>
#include <canopen_bridge/driver/clutch_maxon.hpp>

#include <mmr_edf/mmr_edf.hpp>
#include <mmr_base/configuration.hpp>
#include <mmr_base/msg/cmd_motor.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/qos.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <math.h>
#include <vector>
#include <optional>
#include <algorithm>

class CANOpenBridge : public EDFNode
{
    private:

        std::string m_sInterface;
        int m_nBitrate;
        bool m_bDebug;

        std::string m_sSteerTopic, m_sBrakeTopic, m_sClucthTopic, m_sStatusActuatorTopic, m_sEcuStatusTopic;

        /* Steer parameters */
        int m_nSteerID, m_nVelocity, m_nTimeoutMsgSteer, m_nControlMode, m_nMaxTargetMaxon;
        float m_fWheelRate, m_fIncPerDegree, m_fMinTargetPot;
        float m_fMaxTargetPot, m_fTargetSteerAngle;
        std::optional<float> m_fSteerPot;
        uint32_t m_nCRCSteerOld, m_nCRCSteer;

        /* Brake parameters */
        int m_nBrakeId, m_nMaxTorque, m_nReturnPedalTorque, m_nTimeoutMsgBrake, m_nFreqScaleBrake, m_nCtrBrake = 1;

        /* Clutch parameters */
        int m_nClutchId, m_nVelocityClutch, m_nMonitorClutch, m_nCountClutch = 1, m_nTimeoutMsgClutch;
        std::optional<double> m_fClutchPot;
        bool m_bToDisangaged = false;
        
        std::vector<long int> m_aMotorSteps;
        std::vector<double> m_aPotVal;

        /* Subscriber for CANOpen Command Msg */
        rclcpp::Subscription<mmr_base::msg::CmdMotor>::SharedPtr m_subCmdSteer;
        void msgCmdSteerCallback(mmr_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<mmr_base::msg::CmdMotor>::SharedPtr m_subCmdBrake;
        void msgCmdBrakeCallback(mmr_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<mmr_base::msg::CmdMotor>::SharedPtr m_subCmdClutch;
        void msgCmdClutchCallback(mmr_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr m_subEcuStatus;
        void msgSelectorCallback(mmr_base::msg::EcuStatus::SharedPtr msg);
        void msgEngageInitClutch(mmr_base::msg::EcuStatus::SharedPtr msg);
        void msgEcuStatusCallback(mmr_base::msg::EcuStatus::SharedPtr msg);
        
        void uploadVoltage();

        rclcpp::Publisher<mmr_base::msg::ActuatorStatus>::SharedPtr m_pubActuatorStatus;
        rclcpp::Publisher<mmr_base::msg::ActuatorStatus>::SharedPtr m_pubCANBusTx;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_pubDebugFreq;

        int m_nSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        void connectCANBus();
        void loadParameters();

        inline int getStepToActuate(float fTargetSteerAngle, MOTOR::IDX_TOGGLE_NEW_POS mode) {
            
            int nIncToDo = 0;

            if (mode == MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_ABS_POS)
                return std::round(fTargetSteerAngle * this->m_fIncPerDegree);
            
            if (this->m_fSteerPot.has_value()) {

                fTargetSteerAngle = std::clamp<float>(fTargetSteerAngle, this->m_fMinTargetPot, this->m_fMaxTargetPot);
                float fDeltaDegrees = fTargetSteerAngle - m_fSteerPot.value();
                nIncToDo = std::round(fDeltaDegrees * this->m_fIncPerDegree);
            }

            return nIncToDo;
        }

        MaxonSteer *m_mSteer = nullptr;
        MaxonBrake *m_mBrake = nullptr;
        MaxonClutch *m_mClutch = nullptr;

        mmr_base::msg::ActuatorStatus m_msgActuatorStatus;

    public:

        CANOpenBridge();

        void monitorSteer();
        void sendActuatorStatus();

        ~CANOpenBridge() { close(this->m_nSocket); };

};