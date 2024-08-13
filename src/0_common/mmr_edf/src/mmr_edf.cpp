#include "mmr_edf/mmr_edf.hpp"

void EDFNode::configureEDFScheduler(uint64_t period_ns, uint64_t runtime_ns, uint64_t deadline_ns) {
    // Set the scheduling policy to SCHED_DEADLINE
    sched_attr attr = {
      .size = sizeof(attr),
      .sched_policy = SCHED_DEADLINE,
      .sched_flags = SCHED_FLAG_RESET_ON_FORK,
      .sched_runtime = runtime_ns,
      .sched_deadline = deadline_ns,
      .sched_period = period_ns,
    };

    if (syscall(SYS_sched_setattr, gettid(), &attr, 0) != 0) {
      RCLCPP_ERROR(this->get_logger(), "[ FAILED to SET SCHED_DEADLINE ]: %s", strerror(errno));
      throw std::runtime_error("Wrong parameters for EDF scheduler");
    }
}

bool EDFNode::readGPIOValueFromFile(std::string sFile)
{
    std::ifstream file(sFile);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }

    int nVal;
    file >> nVal;
    return nVal;
}

void EDFNode::writeGPIOValueOnFile(std::string sFile, GPIO_VALUE nValue)
{
    std::ofstream file(sFile);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }

    file << std::to_string((int)nValue);
    if (!file.good()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }
}