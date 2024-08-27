#pragma once

#include <unordered_map>
#include <string_view>
#include <chrono>

namespace RES {

    enum MMR_CAN_MSG_ID {
        /* enable the RES in OpMode using payload = [01, 00, 00, 00, 00, 00, 00, 00] */
        MMR_RES_OPERATIONAL = 0x0,

        /* check the RES Status - [ bit 0 ]: EMERGENCY, [ bit 1 ]: GO_SIGNAL, [ bit 3 ]: BAG [ not used ] */
        MMR_RES_STATUS = 0x191,
    };

    enum MMR_RES_STATUS_MASK {
        RES_SIGNAL_EMERGENCY = 1,
        RES_SIGNAL_GO = 2,
        RES_SIGNAL_BAG = 4
    };

};

namespace ECU {

    enum MMR_CAN_MASK {
        MMR_ECU_MASK = 0x700,
    };

    enum MMR_CAN_MSG_ID {
        
        MMR_STEERING_ANGLE = 0x8A,
        MMR_BRAKING_PERCENTAGE,
        MMR_ACCELERATOR_PERCENTAGE = 0x8c,
        MMR_LAP_COUNTER = 0x91,

        MMR_CLUTCH_PULL_OK = 0xE1,
        MMR_CLUTCH_RELEASE_OK = 0xE3,

        MMR_ECU_PEDAL_THROTTLE = 0x700,
        MMR_ECU_TEMPERATURES,
        MMR_ECU_ENGINE_FN1,
        MMR_ECU_PRESSURES,
        MMR_ECU_ENGINE_FN2,
        MMR_ECU_CLUTCH_STEER,
        MMR_ECU_WHEEL_SPEEDS,
        MMR_ECU_SAFETY_CHECK,
        MMR_ECU_EBS_PRESSURE,
        MMR_ECU_SET_LAUNCH_CONTROL,

        MMR_ECU_GEAR_CONTROL = 0x610,
        MMR_ECU_SET_PIT_LAUNCH = 0x628

    };

    namespace CMD {
        enum ACTIONS {
            GEAR_UP,
            GEAR_DOWN,
            SET_NEUTRAL,
            SET_LAUNCH_CONTROL,
            UNSET_LAUNCH_CONTROL
        };

        const inline std::unordered_map<CMD::ACTIONS, uint8_t> CmdEcuActionsBits {
            { CMD::ACTIONS::GEAR_UP, 7 },
            { CMD::ACTIONS::GEAR_DOWN, 8 },
            { CMD::ACTIONS::SET_LAUNCH_CONTROL, 5 },
            { CMD::ACTIONS::UNSET_LAUNCH_CONTROL, 5 },
            { CMD::ACTIONS::SET_NEUTRAL, 31 },
        };

        struct DATA {
            ECU::MMR_CAN_MSG_ID id;
            uint8_t bit;
        };

    };

    const inline std::unordered_map<CMD::ACTIONS, CMD::DATA> CmdEcuLookup {
        { CMD::ACTIONS::GEAR_UP, {.id = MMR_ECU_GEAR_CONTROL, .bit = CMD::CmdEcuActionsBits.at(CMD::ACTIONS::GEAR_UP)}},
        { CMD::ACTIONS::GEAR_DOWN, {.id = MMR_ECU_GEAR_CONTROL, .bit = CMD::CmdEcuActionsBits.at(CMD::ACTIONS::GEAR_DOWN)}},
        { CMD::ACTIONS::SET_LAUNCH_CONTROL, {.id = MMR_ECU_SET_PIT_LAUNCH, .bit = CMD::CmdEcuActionsBits.at(CMD::ACTIONS::SET_LAUNCH_CONTROL)}},
        { CMD::ACTIONS::UNSET_LAUNCH_CONTROL, {.id = MMR_ECU_SET_PIT_LAUNCH, .bit = CMD::CmdEcuActionsBits.at(CMD::ACTIONS::UNSET_LAUNCH_CONTROL)}},
        { CMD::ACTIONS::SET_NEUTRAL, {.id = MMR_ECU_SET_PIT_LAUNCH, .bit = CMD::CmdEcuActionsBits.at(CMD::ACTIONS::SET_NEUTRAL)}}
    };

};

namespace COCKPIT {

    enum MMR_CAN_MSG_ID {
        MMR_MISSION_SELECTED = 0x40,
        MMR_24V_VOLTAGE = 0x131,
    };

    enum class MMR_MISSION_VALUE {
        MMR_MISSION_IDLE = 0,
        MMR_MISSION_ACCELERATION,
        MMR_MISSION_SKIDPAD,
        MMR_MISSION_AUTOCROSS,
        MMR_MISSION_TRACKDRIVE,
        MMR_MISSION_EBS_TEST,
        MMR_MISSION_INSPECTION,
        MMR_MISSION_MANUAL,
        MMR_MISSION_DEBUG
    };

    const inline std::unordered_map<MMR_MISSION_VALUE, std::string_view> CockpitMissionLookup {
        { MMR_MISSION_VALUE::MMR_MISSION_IDLE, "idle" },
        { MMR_MISSION_VALUE::MMR_MISSION_ACCELERATION, "acceleration"},
        { MMR_MISSION_VALUE::MMR_MISSION_SKIDPAD, "skidpad" },
        { MMR_MISSION_VALUE::MMR_MISSION_AUTOCROSS, "autocross" },
        { MMR_MISSION_VALUE::MMR_MISSION_TRACKDRIVE, "trackdrive" },
        { MMR_MISSION_VALUE::MMR_MISSION_EBS_TEST, "ebs_test" },
        { MMR_MISSION_VALUE::MMR_MISSION_INSPECTION, "inspection" },
        { MMR_MISSION_VALUE::MMR_MISSION_MANUAL, "manual" },
        { MMR_MISSION_VALUE::MMR_MISSION_DEBUG, "debug" },
    };

};

namespace MOTOR {

    enum MMR_CAN_ID_BASE {
        REQUEST_SDO = 0x600,
        RESPONSE_SDO = 0x580,
    };

    enum class ACTUATOR_STATUS{
        DISABLE = 0,
        POSITION_MODE,
        TORQUE_MODE,
        ENGAGE,
        DISENGAGE,
        ERROR,
    };

    enum INDEX_CLUTCH {
        CLUTCH_SET_INIT = 0,
        CLUTCH_SET_DISENGAGED,
        CLUTCH_SET_ENGAGED_1,
        CLUTCH_SET_ENGAGED_2,
        CLUTCH_SET_ENGAGED_3,
        CLUTCH_SET_ENGAGED_4,
    };

    enum MODE_OF_OPERATION {
        HMM = 0x06,
        PPM = 0x01,
        PVM = 0x03,
        CSP = 0x08,
        CSV = 0x09,
        CST = 0x0A,
    };

    enum class IDX_TOGGLE_NEW_POS {
        IDX_WRITE_ABS_POS = 0,
        IDX_WRITE_REL_POS,
    };

};

namespace AS {

    enum STATE {
        OFF = 0,
        CHECKING,
        READY,
        DRIVING,
        FINISHED,
        EMERGENCY
    };

    const inline std::unordered_map<STATE, std::string_view> AsStateStringLookup {
        { STATE::OFF, "OFF" },
        { STATE::CHECKING, "CHECKING"},
        { STATE::READY, "READY" },
        { STATE::DRIVING, "DRIVING" },
        { STATE::FINISHED, "FINISHED" },
        { STATE::EMERGENCY, "EMERGENCY" }
    };

};

namespace timing {
    using namespace std::chrono;

    enum TIME_MODULE {
        MILLISECONDS_MOD = 1000,
        NANOSECONDS_MOD = 1000000000,
    };

    struct Clock {
        template <class duration>
        static inline duration get_time() {
            return duration_cast<duration>(steady_clock::now().time_since_epoch());
        }
    };
};

namespace IMU {
    enum MMR_CAN_MASK {
        MMR_IMU_MASK = 0x400,
    };

    enum MMR_CAN_MSG_ID {
        
        MMR_IMU_ERROR = 0x401,
        MMR_IMU_SAMPLE_TIME = 0x405,
        MMR_IMU_GROUP_COUNTER,
        MMR_IMU_UTC_TIME,
        MMR_IMU_STATUS_WORD = 0x411,
        MMR_IMU_QUATERNION = 0x421,
        MMR_IMU_EULER_ANGLES,
        MMR_IMU_RATE_OF_TURN = 0x432,
        MMR_IMU_ACCELERATION = 0x434,
        MMR_IMU_BAROMETRIC_PRESSURE = 0x452,
        MMR_IMU_LATITUDE_LONGITUDE = 0x471,
        MMR_IMU_VELOCITY = 0x476,
        MMR_IMU_GNSS_STATUS = 0x479,
    };

};