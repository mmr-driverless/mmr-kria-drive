#ifndef CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/msg/res_status.hpp>

#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace estimation {
namespace noop {

class NoopEstimator : public IStateEstimator {
  double m_clutch_disengaged_thresh;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr m_ecu_status_sub;
  rclcpp::Subscription<mmr_base::msg::ResStatus>::SharedPtr m_res_status_sub;

  void odom_cb(std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  void ecu_status_cb(std::shared_ptr<const mmr_base::msg::EcuStatus> msg);
  void res_status_cb(std::shared_ptr<const mmr_base::msg::ResStatus> msg);

public:
  class VehicleState : public IVehicleState {
    std::optional<Eigen::Vector2d> m_position;
    std::optional<double> m_yaw;
    std::optional<bool> m_clutch_is_engaged;
    std::optional<int> m_gear;
    std::optional<double> m_speed;
    std::optional<int> m_rpm;
    std::optional<bool> m_lc_is_active;
    std::optional<bool> m_res_go;
    std::optional<bool> m_res_bag;
    
  public:
    friend NoopEstimator;
    virtual std::optional<Eigen::Vector2d> position() const override { return m_position; }
    virtual std::optional<double> yaw() const override { return m_yaw; }
    virtual std::optional<int> rpm() const override { return m_rpm; }
    virtual std::optional<double> speed() const override { return m_speed; }
    virtual std::optional<bool> lc_is_active() const override { return m_lc_is_active; }
    virtual std::optional<bool> clutch_is_engaged() const override { return m_clutch_is_engaged; }
    virtual std::optional<int> gear() const override { return m_gear; }
    virtual std::optional<bool> res_go() const override { return m_res_go; }
    virtual std::optional<bool> res_bag() const override { return m_res_bag; }
  } m_state;

  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp) override;
  virtual const IVehicleState& update_and_get_current_state() override { return m_state; }
};

}; // namespace noop
}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP