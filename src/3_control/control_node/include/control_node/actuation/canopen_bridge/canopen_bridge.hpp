#ifndef CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP
#define CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP

#include <control_node/actuation/iactuator.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mmr_base/msg/cmd_motor.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <rclcpp/subscription.hpp>

namespace control_node {
namespace actuation {
namespace canopen_bridge {

class CANOpenBridge : public IActuator {
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr m_steer_pub;
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr m_brake_pub;
  rclcpp::Publisher<mmr_base::msg::CmdMotor>::SharedPtr m_clutch_pub;

  rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr m_act_status_sub;

  std::optional<rclcpp::Logger> m_logger;

  bool m_soft_enabled = false;

  enum class EnableRequest {
    None,
    Enable,
    Disable
  };

  class ActuatorStatus {
  public:
    enum Value {
      Unknown = 0,
      Enabled = 1,
      Disabled = 2
    };
    
  private:
    Value m_state;
    static constexpr std::array<const char*, Disabled+2> REPR { "Unknown", "Enabled", "Disabled", "WTF M8" };

  public:
    ActuatorStatus(Value state) : m_state(state) {}
    ActuatorStatus() : m_state(Unknown) {}
    const char* to_string() const { return (m_state >= Enabled && m_state <= Disabled)? REPR[m_state] : REPR.back(); }
    bool is_enabled() const { return m_state == Enabled; }
    bool is_disabled() const { return m_state == Disabled; }
    bool is_unknown() const { return m_state == Unknown; }
    bool operator==(const ActuatorStatus&) const = default;
  };

  EnableRequest m_enable_request;

  struct GroupedActuatorStatus {
    ActuatorStatus steer;
    ActuatorStatus clutch;
    ActuatorStatus brake;
    bool operator==(const GroupedActuatorStatus&) const = default;
  };

  GroupedActuatorStatus m_status;
  std::optional<GroupedActuatorStatus> m_old_status;

  void actuator_status_cb(std::shared_ptr<const mmr_base::msg::ActuatorStatus> msg);
  const rclcpp::Logger& logger() const { return *m_logger; }

  void serve_enable_request();

public:
  virtual void init(rclcpp::Node& node, const Parameters& p, rclcpp::Logger logger) override;
  virtual void actuate(std::chrono::nanoseconds t, const control::Control& control) override;
  ~CANOpenBridge();

  virtual void request_enable() override;
  virtual void request_disable() override;
  virtual bool enabled() const override;
};

};
};
};

#endif // !CONTROLNODE_ACTUATION_CANOPENBRIDGE_CANOPENBRIDGE_HPP