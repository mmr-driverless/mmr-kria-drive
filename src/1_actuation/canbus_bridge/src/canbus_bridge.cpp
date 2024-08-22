#include <canbus_bridge/canbus_bridge.hpp>

CANBusBridge::CANBusBridge() : EDFNode("canbus_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    
    RCLCPP_INFO(
        this->get_logger(),
        "[ INTERFACE ]: %s, [ BITRATE ]: %d, [ DEBUG ]: %d",
        this->m_sInterface.c_str(), this->m_nBitrate, this->m_bDebug
    );

    if (this->m_bDebug)
        RCLCPP_INFO(
            this->get_logger(),
            "[ PERIOD GEAR UP/DOWN ]: %ld, [ PERIOD SET LAUNCH CONTROL ]: %ld, [ PERIOD SET NEUTRAL GEAR ]: %ld",
            this->m_lDelayCmdEcuGear, this->m_lDelayCmdEcuLaunch, this->m_lDelayCmdEcuNeutral
        );

    this->connectCANBus();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


    this->m_subCmdEcuTargetStatus = this->create_subscription<mmr_base::msg::CmdEcu>(
        this->m_sCmdEcuTopic, 1, std::bind(&CANBusBridge::msgCmdEcuCallback, this, std::placeholders::_1));
    this->m_subActuatorsStatus = this->create_subscription<mmr_base::msg::ActuatorStatus>(
        this->m_sActuatorsStatusTopic, 1, std::bind(&CANBusBridge::msgActuatorsStatusCallback, this, std::placeholders::_1));
    this->m_subControlLog = this->create_subscription<mmr_base::msg::ControlLog>(
        this->m_sControlLogTopic, 1, std::bind(&CANBusBridge::msgControlLogCallback, this, std::placeholders::_1));
    this->m_subRaceStatus = this->create_subscription<mmr_base::msg::RaceStatus>(
        this->m_sLapCounterTopic, 1, std::bind(&CANBusBridge::msgRaceStatusCallback, this, std::placeholders::_1));

    this->m_msgEcuStatus.checksum_steering_angle = 0;
    this->m_pubEcuStatus = this->create_publisher<mmr_base::msg::EcuStatus>(this->m_sEcuStatusTopic, qos);
    this->m_pubResStatus = this->create_publisher<mmr_base::msg::ResStatus>(this->m_sResStatusTopic, qos);
    this->m_pubMissionSelect = this->create_publisher<std_msgs::msg::Int8>(this->m_sMissionSelectTopic, 1);
    this->m_pubImuData = this->create_publisher<sensor_msgs::msg::Imu>(this->m_sOutImuDataTopic, qos);

    this->m_ecGearUp.emplace(this->m_nSocket, &this->m_mutexOnSocket, static_cast<int>(this->m_unGearCtrLimit), static_cast<int>(this->m_unGearCtrLimit), this->m_lDelayCmdEcuGear, this->getCanFrame(ECU::CMD::ACTIONS::GEAR_UP));
    this->m_ecGearDown.emplace(this->m_nSocket, &this->m_mutexOnSocket, static_cast<int>(this->m_unGearCtrLimit), static_cast<int>(this->m_unGearCtrLimit), this->m_lDelayCmdEcuGear, this->getCanFrame(ECU::CMD::ACTIONS::GEAR_DOWN));
    this->m_ecSetLaunchCtr.emplace(this->m_nSocket, &this->m_mutexOnSocket, this->m_nLaunchControlCtr, this->m_nLaunchControlCtr, this->m_lDelayCmdEcuLaunch, this->getCanFrame(ECU::CMD::ACTIONS::SET_LAUNCH_CONTROL));
    this->m_ecSetNeutral.emplace(this->m_nSocket, &this->m_mutexOnSocket, this->m_nNeutralCtr, this->m_nNeutralCtr, this->m_lDelayCmdEcuNeutral, this->getCanFrame(ECU::CMD::ACTIONS::SET_NEUTRAL));

}

void CANBusBridge::loadParameters()
{
    declare_parameter("generic.interface", "");
    declare_parameter("generic.bitrate", 5000000);
    declare_parameter("generic.debug", false);
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.max_msgs", 5);
    declare_parameter("generic.control_freq_div", 5);

    declare_parameter("topic.cmdEcuTopic", "");
    declare_parameter("topic.ecuStatusTopic", "");
    declare_parameter("topic.resStatusTopic", "");
    declare_parameter("topic.ActuatorsStatusTopic", "");
    declare_parameter("topic.missionSelectTopic", "");
    declare_parameter("topic.outputImuTopic", "");
    declare_parameter("topic.controlLogTopic", "");
    declare_parameter("topic.raceStatusTopic", "");

    declare_parameter("gear.ctrLimit", 5);
    declare_parameter("gear.changeDeltaTime", 200);
    declare_parameter("gear.delayCmdEcu", 1);

    declare_parameter("launch_control.changeDeltaTime", 100);
    declare_parameter("launch_control.ctrLimit", 2);
    declare_parameter("launch_control.delayCmdEcu", 50);
    
    declare_parameter("neutral.changeDeltaTime", 100);
    declare_parameter("neutral.ctrLimit", 2);
    declare_parameter("neutral.delayCmdEcu", 50);

    get_parameter("generic.interface", this->m_sInterface);
    get_parameter("generic.bitrate", this->m_nBitrate);
    get_parameter("generic.debug", this->m_bDebug);
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.max_msgs", this->m_nMaxMsgs);
    get_parameter("generic.control_freq_div", this->m_nControlFreqDiv);

    get_parameter("topic.cmdEcuTopic", this->m_sCmdEcuTopic);
    get_parameter("topic.ecuStatusTopic", this->m_sEcuStatusTopic);
    get_parameter("topic.resStatusTopic", this->m_sResStatusTopic);
    get_parameter("topic.ActuatorsStatusTopic", this->m_sActuatorsStatusTopic);
    get_parameter("topic.missionSelectTopic", this->m_sMissionSelectTopic);
    get_parameter("topic.outputImuTopic", this->m_sOutImuDataTopic);
    get_parameter("topic.controlLogTopic", this->m_sControlLogTopic);
    get_parameter("topic.raceStatusTopic", this->m_sLapCounterTopic);

    get_parameter("gear.ctrLimit", this->m_unGearCtrLimit);
    get_parameter("gear.changeDeltaTime", this->m_lGearChangeDeltaTime);
    get_parameter("gear.delayCmdEcu", this->m_lDelayCmdEcuGear);

    get_parameter("launch_control.changeDeltaTime", this->m_lLCChangeDeltaTime);
    get_parameter("launch_control.ctrLimit", this->m_nLaunchControlCtr);
    get_parameter("launch_control.delayCmdEcu", this->m_lDelayCmdEcuLaunch);

    get_parameter("neutral.changeDeltaTime", this->m_lNeutralChangeDeltaTime);
    get_parameter("neutral.ctrLimit", this->m_nNeutralCtr);
    get_parameter("neutral.delayCmdEcu", this->m_lDelayCmdEcuNeutral);
}

void CANBusBridge::connectCANBus()
{
    this->m_nSocket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (this->m_nSocket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error on socket define");
        throw 1;
    }

    strcpy(this->m_ifr.ifr_name, this->m_sInterface.c_str());
    ioctl(this->m_nSocket, SIOCGIFINDEX, &this->m_ifr);

    memset(&this->m_addr, 0, sizeof(this->m_addr));
    this->m_addr.can_family = AF_CAN;
    this->m_addr.can_ifindex = this->m_ifr.ifr_ifindex;

    if (bind(this->m_nSocket, (struct sockaddr *)&this->m_addr, sizeof(this->m_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error on socker association");
        throw 1;        
    }

    can_frame frame = {
        .can_id = 0x00,
        .can_dlc = 1,
        .data = { 0x01 }
    };

    this->writeMsg(frame);
}

void CANBusBridge::readMsgFromCANBus()
{
    struct can_frame frame;
    int nMsgRead = 0;

    while (nMsgRead < this->m_nMaxMsgs) {
        
        {
            std::unique_lock<std::mutex> lock(this->m_mutexOnSocket);
            if (read(this->m_nSocket, &frame, sizeof(struct can_frame)) <= 0)
                break;
        }

        if ((frame.can_id & ECU::MMR_ECU_MASK) == ECU::MMR_ECU_MASK)
            this->readEcuStatus(frame);

        else if ((frame.can_id & IMU::MMR_IMU_MASK) == IMU::MMR_IMU_MASK)
            this->readImuStatus(frame);
        
        if (frame.can_id == RES::MMR_RES_STATUS)
            this->readResStatus(frame);

        if (frame.can_id == COCKPIT::MMR_MISSION_SELECTED) {
            std_msgs::msg::Int8 msgMission;
            msgMission.data = frame.data[0];

            this->m_pubMissionSelect->publish(msgMission);
        }

        nMsgRead ++;
    }
}

void CANBusBridge::msgControlLogCallback(const mmr_base::msg::ControlLog::SharedPtr msg)
{

    if ((this->m_nCtrFreqControl % this->m_nControlFreqDiv) != 0) {
        this->m_nCtrFreqControl ++;
        return;
    }
    this->m_nCtrFreqControl = 1;

    float fSteerAngle = static_cast<float>(msg->steer);
    float fBrakePerc = static_cast<float>(msg->brake);
    float fThrottle = static_cast<float>(msg->throttle);

    struct can_frame frame = {
        .can_id = ECU::MMR_STEERING_ANGLE,
        .len = sizeof(float)
    };
    memcpy(frame.data, &fSteerAngle, sizeof(float));
    this->writeMsg(frame);

    frame = {
        .can_id = ECU::MMR_BRAKING_PERCENTAGE,
        .len = sizeof(float)
    };
    memcpy(frame.data, &fBrakePerc, sizeof(float));
    this->writeMsg(frame);

    frame = {
        .can_id = ECU::MMR_ACCELERATOR_PERCENTAGE,
        .len = sizeof(float)
    };
    memcpy(frame.data, &fThrottle, sizeof(float));
    this->writeMsg(frame);
}

void CANBusBridge::msgRaceStatusCallback(const mmr_base::msg::RaceStatus::SharedPtr msg)
{
    uint8_t nLapCounter = msg->current_lap;
    struct can_frame frame = {
        .can_id = ECU::MMR_LAP_COUNTER,
        .len = sizeof(uint8_t),
        .data = { nLapCounter }
    };
    this->writeMsg(frame);
}

void CANBusBridge::changeGearUpDown()
{    
    if (this->m_msgCmdEcu.gear_target == this->m_msgEcuStatus.gear)
        return;

    if ((this->m_msgCmdEcu.gear_target == 0) && (this->m_msgEcuStatus.gear == 1))
        return;

    if ((this->m_msgCmdEcu.gear_target == 1) && (this->m_msgEcuStatus.gear == 0) && (this->m_msgActuatorsStatus.clutch_status != static_cast<uint8_t>(MOTOR::ACTUATOR_STATUS::DISENGAGE)))
        return;

    auto act_time = timing::Clock::get_time<std::chrono::milliseconds>().count();
    if ((this->m_lLastGearTime != 0) && ((act_time - this->m_lLastGearTime) <= this->m_lGearChangeDeltaTime))
        return;

    this->m_lLastGearTime = timing::Clock::get_time<std::chrono::milliseconds>().count();

    ECU::CMD::ACTIONS gear_info;

    gear_info = (this->m_msgCmdEcu.gear_target < this->m_msgEcuStatus.gear) ? 
        ECU::CMD::ACTIONS::GEAR_DOWN :
        ECU::CMD::ACTIONS::GEAR_UP;

    switch (gear_info)
    {
        case ECU::CMD::ACTIONS::GEAR_DOWN:
            if (!this->m_ecGearDown->isRunning())
                this->m_ecGearDown->startCtr();
            break;
            
        case ECU::CMD::ACTIONS::GEAR_UP:
            if (!this->m_ecGearUp->isRunning())
                this->m_ecGearUp->startCtr();
            break;    
    }
}

void CANBusBridge::setGearNeutral() 
{
    if ((this->m_msgCmdEcu.gear_target != 0) || (this->m_msgEcuStatus.gear != 1))
        return;

    if (this->m_msgActuatorsStatus.clutch_status != static_cast<uint8_t>(MOTOR::ACTUATOR_STATUS::DISENGAGE))
        return;

    auto act_time = timing::Clock::get_time<std::chrono::milliseconds>().count();
    if ((this->m_lLastNeutralTime != 0) && ((act_time - this->m_lLastNeutralTime) <= this->m_lNeutralChangeDeltaTime))
        return;

    this->m_lLastNeutralTime = timing::Clock::get_time<std::chrono::milliseconds>().count();

    if ((!this->m_ecSetNeutral->isRunning()) && (!this->m_ecSetLaunchCtr->isRunning()))
        this->m_ecSetNeutral->startCtr();

}

void CANBusBridge::setLaunchControl()
{
    if ((this->m_msgCmdEcu.set_launch_control == this->m_msgEcuStatus.bool_ack_ideal_launch_control))
        return;

    auto act_time = timing::Clock::get_time<std::chrono::milliseconds>().count();
    if ((this->m_lLastLCTime != 0) && ((act_time - this->m_lLastLCTime) <= this->m_lLCChangeDeltaTime))
        return;

    this->m_lLastLCTime = timing::Clock::get_time<std::chrono::milliseconds>().count();

    if ((!this->m_ecSetLaunchCtr->isRunning()) && (!this->m_ecSetNeutral->isRunning()))
        this->m_ecSetLaunchCtr->startCtr();

}

void CANBusBridge::sendStatus()
{
    if (this->m_pubEcuStatus != nullptr) {
        this->m_msgEcuStatus.header.stamp.sec = timing::Clock::get_time<std::chrono::seconds>().count();
        this->m_msgEcuStatus.header.stamp.nanosec = timing::Clock::get_time<std::chrono::nanoseconds>().count() % timing::NANOSECONDS_MOD;
        this->m_msgEcuStatus.header.frame_id = "ECU_STATE";

        this->m_pubEcuStatus->publish(this->m_msgEcuStatus);
    }

    if (this->m_pubResStatus != nullptr) {
        this->m_msgResStatus.header.stamp.sec = timing::Clock::get_time<std::chrono::seconds>().count();
        this->m_msgResStatus.header.stamp.nanosec = timing::Clock::get_time<std::chrono::nanoseconds>().count() % timing::NANOSECONDS_MOD;
        this->m_msgResStatus.header.frame_id = "RES_STATE";

        this->m_pubResStatus->publish(this->m_msgResStatus);
    }
}

void CANBusBridge::readResStatus(can_frame frame)
{
    auto maskRes = [](uint8_t bitvector, RES::MMR_RES_STATUS_MASK mask) -> bool {
        return bitvector & mask;
    };

    this->m_msgResStatus.emergency = !maskRes(frame.data[0], RES::RES_SIGNAL_EMERGENCY);
    this->m_msgResStatus.go_signal = maskRes(frame.data[0], RES::RES_SIGNAL_GO);
    this->m_msgResStatus.bag = maskRes(frame.data[0], RES::RES_SIGNAL_BAG);
}

void CANBusBridge::readEcuStatus(can_frame frame)
{
    switch (frame.can_id)
    {
        case ECU::MMR_ECU_PEDAL_THROTTLE:
            this->m_msgEcuStatus.pot_pedal_a = (float)this->endian_cast<uint16_t>(frame.data) / 1000;
            this->m_msgEcuStatus.pot_pedal_b = (float)this->endian_cast<uint16_t>(frame.data + 2) / 1000;
            this->m_msgEcuStatus.pot_throttle_valve_a = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            this->m_msgEcuStatus.pot_throttle_valve_b = (float)this->endian_cast<uint16_t>(frame.data + 6) / 100;
            break;

        case ECU::MMR_ECU_TEMPERATURES:
            this->m_msgEcuStatus.temp_oil = this->endian_cast<uint16_t>(frame.data) - 40;
            this->m_msgEcuStatus.temp_engine = this->endian_cast<uint16_t>(frame.data + 2) - 40;
            this->m_msgEcuStatus.temp_intake = this->endian_cast<uint16_t>(frame.data + 4) - 40;
            this->m_msgEcuStatus.temp_ambient = this->endian_cast<uint16_t>(frame.data + 6) - 40;
            break;

        case ECU::MMR_ECU_ENGINE_FN1:
            this->m_msgEcuStatus.nmot = this->endian_cast<uint16_t>(frame.data);
            this->m_msgEcuStatus.vehicle_speed = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.gear = this->endian_cast<uint16_t>(frame.data + 4);
            this->m_msgEcuStatus.throttle = (float)this->endian_cast<uint16_t>(frame.data+6) / 100;
            break;
        
        case ECU::MMR_ECU_PRESSURES:
            this->m_msgEcuStatus.p_oil = (float)this->endian_cast<uint16_t>(frame.data) / 20;
            this->m_msgEcuStatus.p_fuel = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.p_intake = (float)this->endian_cast<uint16_t>(frame.data + 4) / 10;
            this->m_msgEcuStatus.p_ambient = (float)this->endian_cast<uint16_t>(frame.data + 6) / 10;
            break;

        case ECU::MMR_ECU_ENGINE_FN2:
            this->m_msgEcuStatus.battery_voltage = (float)this->endian_cast<uint16_t>(frame.data) / 1000;
            this->m_msgEcuStatus.accelerator_pedal = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.fan_control = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            break;

        case ECU::MMR_ECU_CLUTCH_STEER:
            this->m_msgEcuStatus.clutch_percentage = this->endian_cast<float>(frame.data);
            this->m_msgEcuStatus.steering_angle = (float)this->endian_cast<int16_t>(frame.data + 4) / 10;
            this->m_msgEcuStatus.checksum_steering_angle ++;
            this->m_msgEcuStatus.wheel_angle = (float)this->endian_cast<int16_t>(frame.data + 6) / 10;
            break;

        case ECU::MMR_ECU_WHEEL_SPEEDS:
            this->m_msgEcuStatus.wheel_speed_front_left = (float)this->endian_cast<uint16_t>(frame.data) / 100;
            this->m_msgEcuStatus.wheel_speed_front_right = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.wheel_speed_rear_left = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            this->m_msgEcuStatus.wheel_speed_rear_right = (float)this->endian_cast<uint16_t>(frame.data + 6) / 100;
            break;

        case ECU::MMR_ECU_SAFETY_CHECK:
            this->m_msgEcuStatus.p_brake_rear = (float)this->endian_cast<uint16_t>(frame.data) / 200;
            this->m_msgEcuStatus.p_brake_front = (float)this->endian_cast<uint16_t>(frame.data + 2) / 200;
            this->m_msgEcuStatus.error_throttle = (float)this->endian_cast<uint16_t>(frame.data + 4);
            this->m_msgEcuStatus.error_pedal = (float)this->endian_cast<uint16_t>(frame.data + 6);
            break;

        case ECU::MMR_ECU_EBS_PRESSURE:
            this->m_msgEcuStatus.p_ebs_1 = this->endian_cast<float>(frame.data);
            this->m_msgEcuStatus.p_ebs_2 = this->endian_cast<float>(frame.data + 4);
            break;
        
        case ECU::MMR_ECU_SET_LAUNCH_CONTROL:
            this->m_msgEcuStatus.bool_ack_ideal_launch_control = this->endian_cast<uint8_t>(frame.data);
            this->m_msgEcuStatus.bool_ack_real_launch_control = this->endian_cast<uint8_t>(frame.data + 1);
            break;
        
    }
}

void CANBusBridge::readImuStatus(can_frame frame)
{

    this->m_msgOutImuData.header.stamp = this->now();

    switch (frame.can_id)
    {
        case IMU::MMR_IMU_ERROR:
            this->m_msgImuCanData.error_code = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data);
            break;

        case IMU::MMR_IMU_SAMPLE_TIME:
            this->m_msgImuCanData.sample_time = (uint32_t)this->endian_cast<uint32_t, std::endian::big>(frame.data);
            break;

        case IMU::MMR_IMU_GROUP_COUNTER:
            this->m_msgImuCanData.group_counter = (uint16_t)this->endian_cast<uint16_t, std::endian::big>(frame.data);
            break;
        
        case IMU::MMR_IMU_UTC_TIME:
            this->m_msgImuCanData.utc_time.year = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data);
            this->m_msgImuCanData.utc_time.month = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 1);
            this->m_msgImuCanData.utc_time.day = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 2);
            this->m_msgImuCanData.utc_time.hour = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 3);
            this->m_msgImuCanData.utc_time.min = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 4);
            this->m_msgImuCanData.utc_time.sec = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 5);
            this->m_msgImuCanData.utc_time.tenth_ms = (float)this->endian_cast<uint16_t, std::endian::big>(frame.data + 6) * 1e-4;
            break;

        case IMU::MMR_IMU_STATUS_WORD:
            this->m_msgImuCanData.status_word = (uint32_t)this->endian_cast<uint32_t, std::endian::big>(frame.data);
            break;

        case IMU::MMR_IMU_QUATERNION:
            this->m_msgImuCanData.quaternion.qw = (float)this->endian_cast<int16_t, std::endian::big>(frame.data) / 32767;
            this->m_msgImuCanData.quaternion.qx = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 2) / 32767;
            this->m_msgImuCanData.quaternion.qy = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 4) / 32767;
            this->m_msgImuCanData.quaternion.qz = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 6) / 32767;
            setImuOrientation(this->m_msgOutImuData, this->m_msgImuCanData.quaternion.qx, this->m_msgImuCanData.quaternion.qy, this->m_msgImuCanData.quaternion.qz, this->m_msgImuCanData.quaternion.qw);
            break;

        case IMU::MMR_IMU_EULER_ANGLES:
            this->m_msgImuCanData.euler_angles.roll = (float)this->endian_cast<int16_t, std::endian::big>(frame.data) / 128;
            this->m_msgImuCanData.euler_angles.pitch = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 2) / 128;
            this->m_msgImuCanData.euler_angles.yaw = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 4) / 128;
            break;

        case IMU::MMR_IMU_RATE_OF_TURN:
            this->m_msgImuCanData.rate_of_turn.gyro_x = (float)this->endian_cast<int16_t, std::endian::big>(frame.data) / 512;
            this->m_msgImuCanData.rate_of_turn.gyro_y = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 2) / 512;
            this->m_msgImuCanData.rate_of_turn.gyro_z = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 4) / 512;
            setImuAngularVelocity(this->m_msgOutImuData, this->m_msgImuCanData.rate_of_turn.gyro_x, this->m_msgImuCanData.rate_of_turn.gyro_y, this->m_msgImuCanData.rate_of_turn.gyro_z);
            break;

        case IMU::MMR_IMU_ACCELERATION:
            this->m_msgImuCanData.acceleration.acc_x = (float)this->endian_cast<int16_t, std::endian::big>(frame.data) / 256;
            this->m_msgImuCanData.acceleration.acc_y = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 2) / 256;
            this->m_msgImuCanData.acceleration.acc_z = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 4) / 256;
            setImuLinearAcceleration(this->m_msgOutImuData, this->m_msgImuCanData.acceleration.acc_x, this->m_msgImuCanData.acceleration.acc_y, this->m_msgImuCanData.acceleration.acc_z);
            break;
        
        case IMU::MMR_IMU_BAROMETRIC_PRESSURE:
            this->m_msgImuCanData.pressure = (float)this->endian_cast<uint32_t, std::endian::big>(frame.data) / 32768;
            break;
        
        case IMU::MMR_IMU_LATITUDE_LONGITUDE:
            this->m_msgImuCanData.gnss_position.latitude = (float)this->endian_cast<int32_t, std::endian::big>(frame.data) / 16777216;
            this->m_msgImuCanData.gnss_position.longitude = (float)this->endian_cast<int32_t, std::endian::big>(frame.data + 4) / 8388608;
            break;
        
        case IMU::MMR_IMU_VELOCITY:
            this->m_msgImuCanData.velocities.vel_x = (float)this->endian_cast<int16_t, std::endian::big>(frame.data) / 64;
            this->m_msgImuCanData.velocities.vel_y = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 2) / 64;
            this->m_msgImuCanData.velocities.vel_z = (float)this->endian_cast<int16_t, std::endian::big>(frame.data + 4) / 64;
            break;

        case IMU::MMR_IMU_GNSS_STATUS:
            this->m_msgImuCanData.gnss_status.fix_type = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data); 
            this->m_msgImuCanData.gnss_status.n_used_sat = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 1); 
            this->m_msgImuCanData.gnss_status.flags = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 2); 
            this->m_msgImuCanData.gnss_status.date_validity = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 3); 
            this->m_msgImuCanData.gnss_status.n_avail_sat = (uint8_t)this->endian_cast<uint8_t, std::endian::big>(frame.data + 4); 
            break;
    }

    this->m_pubImuData->publish(this->m_msgOutImuData);
}