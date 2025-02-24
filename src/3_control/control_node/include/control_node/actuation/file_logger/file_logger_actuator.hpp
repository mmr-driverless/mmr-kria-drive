#ifndef CONTROLNODE_ACTUATION_FILELOGGER_FILELOGGERACTUATOR_HPP
#define CONTROLNODE_ACTUATION_FILELOGGER_FILELOGGERACTUATOR_HPP

#include "control_node/control/control.hpp"
#include <rclcpp/rclcpp.hpp>

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>
#include <fstream>
#include <filesystem>
#include <stdexcept>

namespace control_node {
namespace actuation {
namespace file_logger {

class FileLoggerActuator : public IActuator {
  bool m_enabled;

  int m_seq_no;

  std::optional<std::ofstream> m_out;

public:
  virtual void init(rclcpp::Node&, const Parameters& p, rclcpp::Logger logger) override {
    auto dir = std::filesystem::path(p.get<std::string>("directory"));
    std::filesystem::create_directory(dir);

    // Create an unique filename in dir
    int idx = 0;
    std::filesystem::path path;
    while (std::filesystem::exists(path = dir / (std::to_string(idx) + ".csv")))
      ++idx;

    m_out.emplace(path);
    if (!m_out->is_open()) {
      RCLCPP_FATAL(logger, "Failed to open/create %s", path.c_str());
      throw std::runtime_error(path.c_str());
    }
    RCLCPP_INFO(logger, "Logging to %s.", path.c_str());
    *m_out << "seq,t_ns,enabled,steer,throttle,brake,clutch,gear,launch\n";
  }
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& u) override {
    *m_out <<
      m_seq_no << "," <<
      t.count() << "," <<
      (m_enabled? 1:0) << "," <<
      u.steer << "," <<
      u.throttle << "," <<
      u.brake << "," <<
      (u.clutch == control::Control::Clutch::Engaged? 1:0) << "," <<
      u.gear << "," <<
      (u.launch == control::Control::LaunchControl::Set? 1:0) << "\n";

    ++m_seq_no;
  }
  ~FileLoggerActuator() { *m_out << "EOF (last seq" << m_seq_no-1 << ")" << std::endl; }

  virtual void request_enable() override { m_enabled = true; };
  virtual void request_disable() override { m_enabled = false; };
  virtual bool enabled() const override { return m_enabled; };
};

}; // namespace logger
}; // namespace actuation
}; // namespace control_node

#endif // !CONTROLNODE_ACTUATION_LOGGER_LOGGERACTUATOR_HPP