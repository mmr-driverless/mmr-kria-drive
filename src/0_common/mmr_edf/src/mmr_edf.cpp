#include "mmr_edf/mmr_edf.hpp"

void EDFNode::configureEDFScheduler(int period_ns, int runtime_ns, int deadline_ns) {
    // Set the scheduling policy to SCHED_DEADLINE
    sched_attr attr = {
      .size = sizeof(attr),
      .sched_policy = SCHED_DEADLINE,
      .sched_runtime = 10 * 1000 * 1000,
      .sched_deadline = 11 * 1000 * 1000,
      .sched_period = 10 * 1000 * 1000 * 1000,
    };

    if (syscall(SYS_sched_setattr, gettid(), &attr, 0) != 0) {
      std::cout << strerror(errno) << std::endl;
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