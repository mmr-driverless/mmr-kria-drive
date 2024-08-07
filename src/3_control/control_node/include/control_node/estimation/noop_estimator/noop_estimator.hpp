#ifndef CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP

#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace control_node {
namespace estimation {
namespace noop {

class NoopEstimator : public IStateEstimator {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg);

public:
  class VehicleState : public IVehicleState {
    std::optional<Eigen::Vector2d> m_position;
    std::optional<double> m_yaw;
    
  public:
    friend NoopEstimator;
    virtual std::optional<Eigen::Vector2d> position() const override { return m_position; }
    virtual std::optional<Eigen::Vector2d> velocity() const override { return std::nullopt; }
    virtual std::optional<double> yaw() const override { return m_yaw; }
    virtual std::optional<double> yaw_rate() const override { return std::nullopt; }
    virtual std::optional<int> rpm() const override { return std::nullopt; }
    virtual std::optional<double> speed() const override { return std::nullopt; }
    virtual std::optional<bool> lc_is_active() const override { return std::nullopt; }
    virtual std::optional<bool> clutch_is_engaged() const override { return std::nullopt; }
    virtual std::optional<int> gear() const override { return std::nullopt; }
  } m_state;

  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp) override;
  virtual const IVehicleState& update_and_get_current_state() override { return m_state; }
};

}; // namespace noop
}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP