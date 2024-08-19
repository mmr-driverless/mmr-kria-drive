#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>

#include <mmr_edf/mmr_edf.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/res_status.hpp>
#include <mmr_base/msg/cmd_ecu.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/msg/imu_can_data.hpp>
#include <mmr_base/msg/control_log.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mmr_base/configuration.hpp>
#include <canbus_bridge/ecu_control.hpp>
#include "imu_helper.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <mutex>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <bit>
#include <algorithm>
#include <cassert>

class CANBusBridge : public EDFNode
{

    private:

        std::string m_sInterface, m_sCmdEcuTopic;
        std::string m_sEcuStatusTopic, m_sResStatusTopic, m_sMissionSelectTopic;
        std::string m_sActuatorsStatusTopic, m_sOutImuDataTopic, m_sControlLogTopic;
        int m_nBitrate, m_nMaxMsgs, m_nControlFreqDiv, m_nCtrFreqControl = 1;
        bool m_bDebug;

        void loadParameters();

        /* Subscriber for target ECU status */
        rclcpp::Subscription<mmr_base::msg::CmdEcu>::SharedPtr m_subCmdEcuTargetStatus;
        void msgCmdEcuCallback(const mmr_base::msg::CmdEcu::SharedPtr msg) { this->m_msgCmdEcu = *msg; }

        /* Subscriber for actuators status */
        rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr m_subActuatorsStatus;
        void msgActuatorsStatusCallback(const mmr_base::msg::ActuatorStatus::SharedPtr msg) { this->m_msgActuatorsStatus = *msg; }

        /* Subscriber for Control Log */
        rclcpp::Subscription<mmr_base::msg::ControlLog>::SharedPtr m_subControlLog;
        void msgControlLogCallback(const mmr_base::msg::ControlLog::SharedPtr msg);

        /* Publisher for CANBus Msg */
        rclcpp::Publisher<mmr_base::msg::EcuStatus>::SharedPtr m_pubEcuStatus;
        rclcpp::Publisher<mmr_base::msg::ResStatus>::SharedPtr m_pubResStatus;

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_pubMissionSelect;

        /* Parsed IMU CAN Data publisher */
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImuData;

        /* message for the pub */
        mmr_base::msg::EcuStatus m_msgEcuStatus;
        mmr_base::msg::ResStatus m_msgResStatus;

        /* Message for IMU Data parsed from CAN Bus*/
        mmr_base::msg::ImuCanData m_msgImuCanData;

        /* Message for output IMU Data */
        sensor_msgs::msg::Imu m_msgOutImuData;

        /* Message for CmdEcu */
        mmr_base::msg::CmdEcu m_msgCmdEcu;

        /* Message for Clutch Actuator Status*/
        mmr_base::msg::ActuatorStatus m_msgActuatorsStatus;

        /* Gear Parameters */
        bool m_bWorkOnGearUpDown = false;
        uint8_t m_unGearCtrLimit;
        long int m_lLastGearTime = 0, m_lGearChangeDeltaTime, m_lDelayCmdEcuGear;

        /* Launch Control Parameters */
        long int m_lLastLCTime = 0, m_lLCChangeDeltaTime, m_lDelayCmdEcuLaunch;
        int m_nLaunchControlCtr;

        /* Set Neutral Parameters */
        long int m_lLastNeutralTime = 0, m_lNeutralChangeDeltaTime, m_lDelayCmdEcuNeutral;
        int m_nNeutralCtr;

        int m_nSocket;
        std::mutex m_mutexOnSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        std::optional<EcuControl> m_ecGearUp, m_ecGearDown, m_ecSetLaunchCtr, m_ecSetNeutral;

        /**
        @param vec Output parameters that represents a string of bytes
        @param n Bit position, numbered from 1, counting from left to right (Ema's notation) 
        */
        inline void toggleNthBit(std::vector<uint8_t> &vec, uint8_t n) {
            n--;  // Shift back to 0-7 range
            uint8_t index = n/8;
            assert(vec.capacity() >= index);
            uint8_t bit = 7 - n%8;
            vec.at(index) ^= ((uint8_t) 1 << bit);
        }

        inline struct can_frame getCanFrame(int nMission) {
            ECU::CMD::DATA info;
            std::vector<uint8_t> data(8);
            std::fill(data.begin(), data.end(), 0);
            info = ECU::CmdEcuLookup.at(static_cast<ECU::CMD::ACTIONS>(nMission));
            this->toggleNthBit(data, info.bit);

            struct can_frame frame = {
                .can_id = info.id,
                .len = 8,
            };

            memcpy(frame.data, &(data.at(0)), data.size());
            return frame;
        }

        inline void writeMsg(struct can_frame frame) {
            std::unique_lock<std::mutex> lock(this->m_mutexOnSocket);
            if (write(this->m_nSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
                return;
        }

        void connectCANBus();
        void readEcuStatus(can_frame frame);
        void readResStatus(can_frame frame);
        void readImuStatus(can_frame frame);

    public:

        CANBusBridge();
        ~CANBusBridge() { close(this->m_nSocket); };

        void readMsgFromCANBus();
        void sendStatus();
        void changeGearUpDown();
        void setGearNeutral();
        void setLaunchControl();
};