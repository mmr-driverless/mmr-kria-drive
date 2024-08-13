#ifndef CONTROLNODE_ESTIMATION_INSPECTIONESTIMATOR_INSPECTIONESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_INSPECTIONESTIMATOR_INSPECTIONESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>

#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace estimation {
namespace inspection {

class InspectionEstimator : public IStateEstimator {
  rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr m_ecu_status_sub;
  rclcpp::Subscription<mmr_base::msg::ActuatorStatus>::SharedPtr m_act_status_sub;

  void ecu_status_cb(std::shared_ptr<const mmr_base::msg::EcuStatus> msg);
  void act_status_cb(std::shared_ptr<const mmr_base::msg::ActuatorStatus> msg);

public:
  class VehicleState : public IVehicleState {
    std::optional<bool> m_clutch_is_engaged;
    std::optional<int> m_gear;
    std::optional<double> m_speed;
    std::optional<int> m_rpm;
    std::optional<bool> m_lc_is_active;
    
  public:
    friend InspectionEstimator;
    virtual std::optional<Eigen::Vector2d> position() const override { return std::nullopt; }
    virtual std::optional<double> yaw() const override { return std::nullopt; }
    virtual std::optional<int> rpm() const override { return m_rpm; }
    virtual std::optional<double> speed() const override { return m_speed; }
    virtual std::optional<bool> lc_is_active() const override { return m_lc_is_active; }
    virtual std::optional<bool> clutch_is_engaged() const override { return m_clutch_is_engaged; }
    virtual std::optional<int> gear() const override { return m_gear; }
  } m_state;

  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp) override;
  virtual const IVehicleState& update_and_get_current_state() override { return m_state; }
};

}; // namespace inspection
}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_INSPECTIONESTIMATOR_INSPECTIONESTIMATOR_HPP