#include <mmr_data_logger/mmr_data_logger.hpp>

MMR_Data_Logger::load_parameters(){
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

    get_parameter("generic.WCET", this->m_nWCET);
    get_parameter("generic.period", this->m_nPeriod);
    get_parameter("generic.deadline", this->m_nDeadline);

    get_parameter("topics.statusActuatorTopic", this->subActuator);
    get_parameter("topics.ecuStatusTopic", this->subEcu);
    get_parameter("topics.xsenseTopic", this->subImu);
    get_parameter("topics.asTopic", this->subAsState);
    get_parameter("topics.missionTopic", this->subMissionSelected);
    get_parameter("topics.lapCounterTopic", this->subRaceStatus);
    get_parameter("topics.conesActualTopic", this->subConesActual);
    get_parameter("topics.conesAllTopic", this->subConesAll);
    get_parameter("topics.controlTopic", this->subControl);
}

MMR_Data_Logger::MMR_Data_Logger(): 
EDFNode("MMR_Data_Logger")
{
    this->load_parameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);

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

    this->subAsState= this->create_subscription<std_msgs::msg::UInt8>(
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

    this->subControl= this->create_subscription<mmr_base::msg::Marker>(
      this->controlTopic, bestEffortQOS, std::bind(&MMR_Data_Logger::controlCallBack, this, _1)
    );
   
   
}