#include <canopen_bridge/canopen_bridge.hpp>

CANOpenBridge::CANOpenBridge() : EDFNode("canopen_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    this->connectCANBus();
    
    if (this->m_bDebug) {
        RCLCPP_INFO(this->get_logger(), "[ INFO ] CAN INTERFACE: %s", this->m_sInterface.c_str());
        RCLCPP_INFO(this->get_logger(), "[ INFO ] CAN BITRATE: %d", this->m_nBitrate);
        RCLCPP_INFO(this->get_logger(), "[ INFO ] MONITOR FREQUENCY CLUTCH: %d", this->m_nMonitorClutch);
    }

    this->m_subCmdSteer = this->create_subscription<mmr_base::msg::CmdMotor>(
        this->m_sSteerTopic, 5, std::bind(&CANOpenBridge::msgCmdSteerCallback, this, std::placeholders::_1));

    this->m_subCmdBrake = this->create_subscription<mmr_base::msg::CmdMotor>(
        this->m_sBrakeTopic, 5, std::bind(&CANOpenBridge::msgCmdBrakeCallback, this, std::placeholders::_1));

    this->m_subCmdClutch = this->create_subscription<mmr_base::msg::CmdMotor>(
        this->m_sClucthTopic, 5, std::bind(&CANOpenBridge::msgCmdClutchCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    this->m_subEcuStatus = this->create_subscription<mmr_base::msg::EcuStatus>(
        this->m_sEcuStatusTopic, qos, std::bind(&CANOpenBridge::msgSelectorCallback, this, std::placeholders::_1));

    this->m_pubActuatorStatus = this->create_publisher<mmr_base::msg::ActuatorStatus>(m_sStatusActuatorTopic, 1);
    this->m_pubDebugFreq = this->create_publisher<std_msgs::msg::Int8>("/debug/clutch", 1);

    this->m_msgActuatorStatus.brake_status  = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    this->m_msgActuatorStatus.steer_status  = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);

    this->m_fConvFactor = (this->m_fMaxTargetMaxon - (this->m_fMinTargetPot)) / (this->m_fMaxTargetPot - (-this->m_fMaxTargetPot));
    this->m_nCRCSteerOld = 0;
}

void CANOpenBridge::loadParameters()
{
    declare_parameter("generic.interface", "");
    declare_parameter("generic.bitrate", 5000000);
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.debug", false);

    declare_parameter("topic.steerTopic", "");
    declare_parameter("topic.brakeTopic", "");
    declare_parameter("topic.clutchTopic", "");
    declare_parameter("topic.statusActuatorTopic", "");
    declare_parameter("topic.ecuStatusTopic", "");

    declare_parameter("steer.node_id", 18);
    declare_parameter("steer.wheel_rate", 6.4286);
    declare_parameter("steer.inc_per_degree", 179.7224);
    declare_parameter("steer.max_target_pot", 135.0);
    declare_parameter("steer.max_target_maxon", 24000.0);
    declare_parameter("steer.velocity", 2750);
    declare_parameter("steer.timeout_msgs", 5);
    declare_parameter("steer.control_mode", 0);
    declare_parameter("steer.min_target_pot", 0.0);

    declare_parameter("brake.node_id", 18);
    declare_parameter("brake.max_torque", 1500);
    declare_parameter("brake.return_pedal_torque", -20);
    declare_parameter("brake.timeout_msgs", 5);
    declare_parameter("brake.monitor_freq", 5);

    declare_parameter("clutch.node_id", 16);
    declare_parameter("clutch.velocity", 3500);
    declare_parameter("clutch.monitoring_freq", 10);
    declare_parameter("clutch.timeout_msgs", 2);
    declare_parameter<std::vector<long int>>("clutch.step_maxon", std::vector<long int>());
    declare_parameter<std::vector<double>>("clutch.pot_val", std::vector<double>());
    
    get_parameter("generic.interface", this->m_sInterface);
    get_parameter("generic.bitrate", this->m_nBitrate);
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.debug", this->m_bDebug);

    get_parameter("topic.steerTopic", this->m_sSteerTopic);
    get_parameter("topic.brakeTopic", this->m_sBrakeTopic);
    get_parameter("topic.clutchTopic", this->m_sClucthTopic);
    get_parameter("topic.statusActuatorTopic", this->m_sStatusActuatorTopic);
    get_parameter("topic.ecuStatusTopic", this->m_sEcuStatusTopic);

    get_parameter("steer.node_id", this->m_nSteerID);
    get_parameter("steer.wheel_rate", this->m_fWheelRate);
    get_parameter("steer.inc_per_degree", this->m_fIncPerDegree);
    get_parameter("steer.max_target_pot", this->m_fMaxTargetPot);
    get_parameter("steer.min_target_pot", this->m_fMinTargetPot);
    get_parameter("steer.max_target_maxon", this->m_fMaxTargetMaxon);
    get_parameter("steer.velocity", this->m_nVelocity);
    get_parameter("steer.timeout_msgs", this->m_nTimeoutMsgSteer);
    get_parameter("steer.control_mode", this->m_nControlMode);

    get_parameter("brake.node_id", this->m_nBrakeId);
    get_parameter("brake.max_torque", this->m_nMaxTorque);
    get_parameter("brake.return_pedal_torque", this->m_nReturnPedalTorque);
    get_parameter("brake.timeout_msgs", this->m_nTimeoutMsgBrake);
    get_parameter("brake.monitor_freq", this->m_nFreqScaleBrake);

    get_parameter("clutch.node_id", this->m_nClutchId);
    get_parameter("clutch.velocity", this->m_nVelocityClutch);
    get_parameter("clutch.monitoring_freq", this->m_nMonitorClutch);
    get_parameter("clutch.step_maxon", this->m_aMotorSteps);
    get_parameter("clutch.pot_val", this->m_aPotVal);
    get_parameter("clutch.timeout_msgs", this->m_nTimeoutMsgClutch);
}

void CANOpenBridge::connectCANBus()
{
    this->m_nSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
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

    /* send OPERATIONAL MODE on CanOpen Network */
    can_frame frame = {
        .can_id = 0x00,
        .can_dlc = 2,
        .data = {0x01, 0x12}
    };

    if (write(this->m_nSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        return;
}

void CANOpenBridge::msgCmdSteerCallback(mmr_base::msg::CmdMotor::SharedPtr msg)
{
    if (msg->enable && (this->m_mSteer == nullptr)) {
        /* Enables the steer motor in PPM */
        this->m_mSteer = new MaxonSteer(
            this->m_nSocket, this->m_nSteerID, this->m_nTimeoutMsgSteer,
            this->m_fMaxTargetMaxon, this->m_nVelocity
        );
        this->m_msgActuatorStatus.steer_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::POSITION_MODE);

        return;
    }

    if (msg->disable && (this->m_mSteer != nullptr)) {
        delete this->m_mSteer;
        this->m_mSteer = nullptr;
        this->m_msgActuatorStatus.steer_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    }

    if (msg->homing) {
        this->m_mSteer = new MaxonSteer(this->m_nSocket, m_nSteerID);
        delete this->m_mSteer;
        this->m_mSteer = nullptr;
        return;
    }

    this->m_fTargetWheelAngle = msg->wheel_angle;
}

void CANOpenBridge::msgCmdBrakeCallback(mmr_base::msg::CmdMotor::SharedPtr msg)
{
    
    if (msg->enable && (this->m_mBrake == nullptr)) {
        /* Enables the brake motor in CST */
        this->m_mBrake = new MaxonBrake(
            this->m_nSocket, this->m_nBrakeId, this->m_nTimeoutMsgBrake,
            this->m_nMaxTorque, m_nReturnPedalTorque
        );

        uint32_t nMaxTorqueNominal = this->m_mBrake->upload<uint32_t>(0x6076, 0x00);
        uint32_t nNominalCurrent = this->m_mBrake->upload<uint32_t>(0x3031, 0x01);
        if (this->m_bDebug)
            RCLCPP_INFO(
                this->get_logger(), "[ INFO ] ENABLE RECEIVED FOR BRAKE, [ MAX TORQUE ]: %u uNm, [ NOMINAL CURRENT ]: %u mA", 
                nMaxTorqueNominal, nNominalCurrent 
            );
        
        this->m_msgActuatorStatus.brake_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::TORQUE_MODE);
        return;
    }

    if (msg->disable && (this->m_mBrake != nullptr)) {
        delete this->m_mBrake;
        this->m_mBrake = nullptr;
        this->m_msgActuatorStatus.brake_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    }
    
    if (this->m_bDebug)
        RCLCPP_INFO(this->get_logger(), "[ TORQUE REQUEST ]: %lf", msg->brake_torque);

    if (this->m_mBrake != nullptr)
        this->m_mBrake->writeTargetTorque(msg->brake_torque);
}

void CANOpenBridge::msgCmdClutchCallback(mmr_base::msg::CmdMotor::SharedPtr msg)
{
    if (msg->enable && (this->m_mClutch == nullptr)) {
        this->m_mClutch = new MaxonClutch(
            this->m_nSocket, this->m_nClutchId, 
            this->m_nTimeoutMsgClutch, this->m_nVelocityClutch,
            this->m_aMotorSteps, this->m_aPotVal
        );
        
        if (this->m_bDebug)
            RCLCPP_INFO(this->get_logger(), "[ INFO ] ENABLE RECEIVED FOR CLUTCH");

        return;
    }

    if (msg->disable && (this->m_mClutch != nullptr)) { 
        delete this->m_mClutch;
        this->m_mClutch = nullptr;
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    }

    this->m_bToDisangaged = msg->disengaged;
    
    if (this->m_bDebug)
        RCLCPP_INFO(this->get_logger(), "[ SET TO DISANGAGED ]: %d", this->m_bToDisangaged);
}

void CANOpenBridge::msgSelectorCallback(mmr_base::msg::EcuStatus::SharedPtr msg)
{
    this->m_fSteerPot = msg->steering_angle;
    this->m_nCRCSteer = msg->checksum_steering_angle;

    if (this->m_fClutchPot.has_value())
        this->msgEcuStatusCallback(msg);
    else this->msgEngageInitClutch(msg);
}

void CANOpenBridge::msgEngageInitClutch(mmr_base::msg::EcuStatus::SharedPtr msg)
{
    float fClutchPot = msg->clutch_percentage;
    if (this->m_mClutch == nullptr)
        return;
    
    if (fClutchPot > this->m_aPotVal[MOTOR::CLUTCH_SET_INIT]){
        this->m_mClutch->engage(static_cast<int>(this->m_aMotorSteps[MOTOR::CLUTCH_SET_INIT]));
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISENGAGE);
    }
    else{
        this->m_fClutchPot = fClutchPot;
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::ENGAGE);
    }
}

void CANOpenBridge::msgEcuStatusCallback(mmr_base::msg::EcuStatus::SharedPtr msg)
{
    float fClutchPot = msg->clutch_percentage;

    /* need to slow down the frequency of control to this->m_nPeriod [ hz ] to (this->m_nPeriod / this->m_nMonitorClutch) [ hz ]  */
    if ((this->m_nCountClutch % this->m_nMonitorClutch) != 0) {
        this->m_nCountClutch ++;
        return;
    }

    this->m_nCountClutch = 1;

    if (this->m_bDebug) {
        std_msgs::msg::Int8 msg;
        msg.data = 1;
        this->m_pubDebugFreq->publish(msg);
    }

    if (
        ((fClutchPot < this->m_aPotVal[MOTOR::CLUTCH_SET_ENGAGED_4]) && (!this->m_bToDisangaged)) || 
        ((fClutchPot > this->m_aPotVal[MOTOR::CLUTCH_SET_DISENGAGED]) && (this->m_bToDisangaged))
    ) {
        /* set the status of clutch ENGAGE to DISENGAGE*/
        this->m_msgActuatorStatus.clutch_status = (fClutchPot < this->m_aPotVal[MOTOR::CLUTCH_SET_ENGAGED_4]) ? static_cast<unsigned int>(MOTOR::ACTUATOR_STATUS::ENGAGE) : static_cast<unsigned int>(MOTOR::ACTUATOR_STATUS::DISENGAGE);

        if (this->m_bDebug)
            RCLCPP_INFO(this->get_logger(), "[ INFO ]: no need to change status");

        return;
    }
    
    if (this->m_bDebug)
        RCLCPP_INFO(this->get_logger(), "[ POT VALUE ]: %f, [ TO DISENGAGED ]: %d", fClutchPot, this->m_bToDisangaged);

    if (!this->m_bToDisangaged) {
        
        if (this->m_bDebug)
            RCLCPP_INFO(this->get_logger(), "[ INFO ]: engaged clutch");

        if (this->m_mClutch != nullptr)
            this->m_mClutch->engage(fClutchPot);
    }
    else { 
    
        if (this->m_bDebug)
            RCLCPP_INFO(this->get_logger(), "[ INFO ]: disengaged clutch");
        
        if (this->m_mClutch != nullptr)
            this->m_mClutch->disengage(fClutchPot);
    }
}

void CANOpenBridge::sendActuatorStatus()
{
    if ((this->m_nCtrBrake % this->m_nFreqScaleBrake) == 0) {
        if (this->m_mBrake != nullptr)
            this->uploadVoltage();
        this->m_nCtrBrake = 1;
    }
    else this->m_nCtrBrake ++;

    this->m_msgActuatorStatus.header.stamp.sec = timing::Clock::get_time<std::chrono::seconds>().count();
    this->m_msgActuatorStatus.header.stamp.nanosec = timing::Clock::get_time<std::chrono::nanoseconds>().count() % timing::NANOSECONDS_MOD;
    this->m_pubActuatorStatus->publish(this->m_msgActuatorStatus); 
}

void CANOpenBridge::monitorSteer()
{
    if (static_cast<MOTOR::IDX_TOGGLE_NEW_POS>(this->m_nControlMode) == MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_ABS_POS) {
        if (this->m_nCRCSteerOld != this->m_nCRCSteer)
            this->m_nCRCSteerOld = this->m_nCRCSteer;
        else return;
    }
    /* Compute the incremets to do */
    int nIncrements = this->getStepToActuate(m_fTargetWheelAngle, static_cast<MOTOR::IDX_TOGGLE_NEW_POS>(this->m_nControlMode));
    
    if ((this->m_bDebug) && (this->m_fSteerPot.has_value())) {
        RCLCPP_INFO(
            this->get_logger(), 
            "[ STEERING ANGLE POT ]: %f, [ WHEEL ANGLE TARGET ]: %f, [ NUMBER INCREMENT ]: %d",
            this->m_fSteerPot.value(), m_fTargetWheelAngle, nIncrements
        );
    }
    
    if (this->m_mSteer != nullptr)  
        this->m_mSteer->writeTargetPos(nIncrements, static_cast<MOTOR::IDX_TOGGLE_NEW_POS>(this->m_nControlMode));
}

void CANOpenBridge::uploadVoltage()
{
    uint16_t m_uVoltage = this->m_mBrake->upload<uint16_t>(0x2200, 0x01);
    this->m_msgActuatorStatus.voltage = ( (float) m_uVoltage / 10);
}