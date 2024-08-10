#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <can_msgs/msg/frame.hpp>

#include <mmr_edf/mmr_edf.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/res_status.hpp>
#include <mmr_base/msg/cmd_ecu.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <std_msgs/msg/int8.hpp>
#include <mmr_base/configuration.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <bit>
#include <algorithm>
#include <cassert>

class CANBusBridge : public EDFNode
{

    private:

        std::string m_sInterface, m_sTopicTx, m_sTopicRx, m_sCmdEcuTopic, m_sEcuStatusTopic, m_sResStatusTopic, m_sMissionSelectTopic, m_sActuatorsStatusTopic;
        int m_nBitrate, m_nMaxMsgs;
        bool m_bDebug;

        void loadParameters();

        /* Subscriber for CANBus Msg */
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCANRx;
        void msgCANBusRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

        /* Subscriber for target ECU status */
        rclcpp::Subscription<mmr_base::msg::CmdEcu>::SharedPtr m_subCmdEcuTargetStatus;
        void msgCmdEcuCallback(const mmr_base::msg::CmdEcu::SharedPtr msg) { this->m_msgCmdEcu = *msg; }

        /* Subscriber for actuators status */
        rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr m_subActuatorsStatus;
        void msgActuatorsStatusCallback(const mmr_base::msg::ActuatorStatus::SharedPtr msg) { this->m_msgActuatorsStatus = *msg; }

        /* Publisher for CANBus Msg */
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_pubCANBusTx;
        rclcpp::Publisher<mmr_base::msg::EcuStatus>::SharedPtr m_pubEcuStatus;
        rclcpp::Publisher<mmr_base::msg::ResStatus>::SharedPtr m_pubResStatus;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_pubMissionSelect;

        /* message for the pub */
        mmr_base::msg::EcuStatus m_msgEcuStatus;
        mmr_base::msg::ResStatus m_msgResStatus;

        /* Message for CmdEcu */
        mmr_base::msg::CmdEcu m_msgCmdEcu;

        /* Message for Clutch Actuator Status*/
        mmr_base::msg::ActuatorStatus m_msgActuatorsStatus;

        /* Gear Parameters */
        uint8_t m_unGearCtrLimit;
        long int m_lLastGearTime = 0, m_lGearChangeDeltaTime;

        /* Launch Control Parameters */
        long int m_lLastLCTime = 0, m_lLCChangeDeltaTime;
        bool m_bSetLCValue = false;

        /* Set Neutral Parameters */
        long int m_lLastNeutralTime = 0, m_lNeutralChangeDeltaTime;
        bool m_bSetNeutralValue = false;

        int m_nSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        /**
        @param vec output parameters that represents a string of byts
        @param n Bit position, numbered from 1, counting from left to right (Ema's notation) 
        */
        inline void toggleNthBit(std::vector<uint8_t> &vec, uint8_t n) {
            n--;  // Shift back to 0-7 range
            uint8_t index = i/8;
            assert(vec.capacity() >= index);
            uint8_t bit = 7 - i%8;
            vec.at(index) ^= ((uint8_t) 1 << bit);
        }

        void connectCANBus();
        void readEcuStatus(can_frame frame);
        void readResStatus(can_frame frame);

    public:

        CANBusBridge();
        ~CANBusBridge() { close(this->m_nSocket); };

        void readMsgFromCANBus();
        void sendStatus();
        void changeGearUpDown();
        void setGearNeutral();
        void setLaunchControl();
};