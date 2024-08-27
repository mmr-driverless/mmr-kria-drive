#include <mmr_data_logger/mmr_data_logger.hpp>
#include <sstream>

void MMR_Data_Logger::load_parameters(){
    declare_parameter("generic.WCET", 5000000);
    declare_parameter("generic.period", 10000000);
    declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.debug", false);

    declare_parameter("topics.statusActuatorTopic", "");
    declare_parameter("topics.ecuStatusTopic", "");
    declare_parameter("topics.xsenseTopic", "");
    declare_parameter("topics.asTopic", "");
    declare_parameter("topics.missionTopic", "");
    declare_parameter("topics.lapCounterTopic", "");
    declare_parameter("topics.conesActualTopic", "");
    declare_parameter("topics.conesAllTopic", "");
    declare_parameter("topics.controlTopic", "");
    declare_parameter("topics.sendMsgTopic", "");


    get_parameter("generic.WCET", this->m_nWCET);
    get_parameter("generic.period", this->m_nPeriod);
    get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.debug", this->debug);

    get_parameter("topics.statusActuatorTopic", this->statusActuatorTopic);
    get_parameter("topics.ecuStatusTopic", this->ecuStatusTopic);
    get_parameter("topics.xsenseTopic", this->xsenseTopic);
    get_parameter("topics.asTopic", this->asTopic);
    get_parameter("topics.missionTopic", this->missionTopic);
    get_parameter("topics.lapCounterTopic", this->lapCounterTopic);
    get_parameter("topics.conesActualTopic", this->conesActualTopic);
    get_parameter("topics.conesAllTopic", this->conesAllTopic);
    get_parameter("topics.controlTopic", this->controlTopic);
    get_parameter("topics.sendMsgTopic", this->sendMsgTopic);

}

MMR_Data_Logger::MMR_Data_Logger(): 
EDFNode("MMR_Data_Logger")
{
    this->load_parameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

    this->pubMsg=this->create_publisher<can_msgs::msg::Frame>(this->sendMsgTopic, 1);

    using namespace std::placeholders;
    const auto &bestEffortQOS = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    this->subEcu= this->create_subscription<mmr_base::msg::EcuStatus>(
      this->ecuStatusTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::ecuCallBack, this, _1)
    );

    this->subActuator= this->create_subscription<mmr_base::msg::ActuatorStatus>(
      this->statusActuatorTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::actuatorStatusCallBack, this, _1)
    );

    this->subImu= this->create_subscription<sensor_msgs::msg::Imu>(
      this->xsenseTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::imuCallBack, this, _1)
    );

    this->subAsState= this->create_subscription<std_msgs::msg::Int8>(
      this->asTopic, 1, std::bind(&MMR_Data_Logger::asStateCallBack, this, _1)
    );

    this->subMissionSelected= this->create_subscription<std_msgs::msg::Int8>(
      this->missionTopic, 1, std::bind(&MMR_Data_Logger::missionSelectedCallBack, this, _1)
    );

    this->subRaceStatus= this->create_subscription<mmr_base::msg::RaceStatus>(
      this->lapCounterTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::raceStatusCallBack, this, _1)
    );

    this->subConesActual= this->create_subscription<mmr_base::msg::Marker>(
      this->conesActualTopic, 1, std::bind(&MMR_Data_Logger::conesActualCallBack, this, _1)
    );
    
    this->subConesAll= this->create_subscription<mmr_base::msg::Marker>(
      this->conesAllTopic, 10, std::bind(&MMR_Data_Logger::conesAllCallBack, this, _1)
    );

    this->subControl= this->create_subscription<mmr_base::msg::ControlLog>(
      this->controlTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::controlCallBack, this, _1)
    );
   
   
}


MMR_Data_Logger::~MMR_Data_Logger() {
  RCLCPP_INFO(this->get_logger(), "Destroying MMR_Data_Logger...");
  rclcpp::shutdown();
}

uint64_t MMR_Data_Logger::pack_bits(uint64_t value, int position, int length) {
    return (value & ((1ULL << length) - 1)) << position;
}

uint64_t  MMR_Data_Logger::create_dv_driving_dynamics_1_message() {
    uint64_t message = 0;

    // Impacchettamento dei dati nei bit corretti secondo la tabella
    message |= pack_bits(this->speedActual, 0, 8);                 // bit 0-7
    message |= pack_bits(this->speedTarget, 8, 8);                 // bit 8-15
    message |= pack_bits(this->steeringAgleActual, 16, 8);// bit 16-23
    message |= pack_bits(this->steeringAngleTarget, 24, 8);// bit 24-31
    message |= pack_bits(this->brakeActual, 32, 8);            // bit 32-39
    message |= pack_bits(this->brakeTarget, 40, 8);            // bit 40-47
    message |= pack_bits(0, 48, 8);          // bit 48-55
    message |= pack_bits(0, 56, 8);        // bit 56-63

    return message;
}


uint64_t  MMR_Data_Logger::create_dv_driving_dynamics_2_message() {
    uint64_t message = 0;

    // Impacchettamento dei dati nei bit corretti secondo la tabella
    message |= pack_bits(this->accelerationLongitudinal, 0, 16);                 // bit 0-15
    message |= pack_bits(this->accelerationLateral, 16, 16);                 // bit 16-31
    message |= pack_bits(this->yawRate, 32, 16);// bit 32-47
    return message;
}

uint64_t  MMR_Data_Logger::create_dv_system_status_messagge() {
    uint64_t message = 0;

    // Impacchettamento dei dati nei bit corretti secondo la tabella
    message |= pack_bits(this->asStatus, 0, 3);                 // bit 0-2
    message |= pack_bits(this->ebsState, 3, 2);                 // bit 3-4
    message |= pack_bits(this->missionSelected, 5, 3);// bit 5-7
    message |= pack_bits(this->steeringState, 8, 1);// bit 8
    message |= pack_bits(this->serviceBrakeState, 9, 2);            // bit 9-10
    message |= pack_bits(this->lapCounter, 11, 4);            // bit 11-14
    message |= pack_bits(this->conesCountActual, 15, 8);          // bit 15-22
    message |= pack_bits(this->lapCounter, 23, 17);        // bit 23-39

    return message;
}

void MMR_Data_Logger::send_messages(){

  //print_parameters();

  uint64_t message_500 = create_dv_driving_dynamics_1_message();
  auto msg_500 = can_msgs::msg::Frame();
  msg_500.id = 0x500;
  msg_500.dlc = 8;
  *(uint64_t*)msg_500.data.begin() = message_500;
    
  this->pubMsg->publish(msg_500);

  uint64_t message_501 = create_dv_driving_dynamics_2_message();
  auto msg_501 = can_msgs::msg::Frame();
  msg_501.id = 0x501;
  msg_501.dlc = 6;
  *(uint64_t*)msg_501.data.begin() = message_501;
  this->pubMsg->publish(msg_501);


  uint64_t message_502 = create_dv_system_status_messagge();
  auto msg_502 = can_msgs::msg::Frame();
  msg_502.id = 0x502;
  msg_502.dlc = 5;
  *(uint64_t*)msg_502.data.begin() = message_502;
  this->pubMsg->publish(msg_502);
}


void MMR_Data_Logger::print_parameters(){
    std::ostringstream oss;

    // Header
    oss << "Vehicle Data Logging:\n";

    // Pressure Values
    oss << "Pressure Values:\n";
    oss << "  P Brake Rear:  " << this->pbrake_rear << "\n";
    oss << "  P Brake Front: " << this->pbrake_front << "\n";
    oss << "  P EBS1:        " << this->pebs1 << "\n";
    oss << "  P EBS2:        " << this->pebs2 << "\n";

    // DV driving dynamics 1
    oss << "DV Driving Dynamics 1:\n";
    oss << "  Speed Target:        " << static_cast<int>(this->speedTarget) << "\n";
    oss << "  Speed Actual:        " << static_cast<int>(this->speedActual) << "\n";
    oss << "  Brake Actual:        " << static_cast<int>(this->brakeActual) << "\n";
    oss << "  Brake Target:        " << static_cast<int>(this->brakeTarget) << "\n";
    oss << "  Steering Angle Actual: " << static_cast<int>(this->steeringAgleActual) << "\n";
    oss << "  Steering Angle Target: " << static_cast<int>(this->steeringAngleTarget) << "\n";

    // DV driving dynamics 2
    oss << "DV Driving Dynamics 2:\n";
    oss << "  Acceleration Longitudinal: " << this->accelerationLongitudinal << "\n";
    oss << "  Acceleration Lateral:      " << this->accelerationLateral << "\n";
    oss << "  Yaw Rate:                  " << this->yawRate << "\n";

    // DV system status
    oss << "DV System Status:\n";
    oss << "  AS Status:        " << static_cast<int>(this->asStatus) << "\n";
    oss << "  EBS State:        " << static_cast<int>(this->ebsState) << "\n";
    oss << "  Mission Selected: " << static_cast<int>(this->missionSelected) << "\n";
    oss << "  Service Brake State: " << static_cast<int>(this->serviceBrakeState) << "\n";
    oss << "  Steering State:      " << (this->steeringState ? "True" : "False") << "\n";
    oss << "  Lap Counter:         " << static_cast<int>(this->lapCounter) << "\n";
    oss << "  Cones Count Actual:  " << static_cast<int>(this->conesCountActual) << "\n";
    oss << "  Cones Count All:     " << this->conesCountAll << "\n";

    // Print all in one go
    RCLCPP_INFO(rclcpp::get_logger("vehicle_logger"), "%s", oss.str().c_str());
}
