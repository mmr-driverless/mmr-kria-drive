#ifndef CONTROLNODE_ESTIMATION_SIMESTIMATOR_SIMESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_SIMESTIMATOR_SIMESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <mmr_base/msg/ecu_status.hpp>
#include <mmr_base/msg/actuator_status.hpp>
#include <mmr_base/msg/res_status.hpp>

#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/parameters.hpp>

namespace control_node {
namespace estimation {
namespace sim {

class SimEstimator : public IStateEstimator {

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

  void odom_cb(std::shared_ptr<const nav_msgs::msg::Odometry> msg);

public:
  class VehicleState : public IVehicleState {
    std::optional<Eigen::Vector2d> m_position;
    std::optional<double> m_yaw;
    std::optional<double> m_speed;
    std::optional<double> m_yaw_rate;
    std::optional<double> m_vy;
    
  public:
    friend SimEstimator;
    virtual std::optional<Eigen::Vector2d> position() const override { return m_position; }
    virtual std::optional<double> yaw() const override { return m_yaw; }
    virtual std::optional<double> speed() const override { return m_speed; }
    virtual std::optional<double> vy() const override { return m_vy; }
    virtual std::optional<double> yaw_rate() const override { return m_yaw_rate; }
    
    // don't need these but forced to implement them
    virtual std::optional<int> rpm() const override { return std::nullopt;}
    virtual std::optional<bool> lc_is_active() const override { return std::nullopt; }
    virtual std::optional<bool> clutch_is_engaged() const override { return std::nullopt; }
    virtual std::optional<int> gear() const override { return std::nullopt;}
    virtual std::optional<bool> res_go() const override { return std::nullopt; }
    virtual std::optional<bool> res_bag() const override { return std::nullopt; }
    virtual std::optional<double> actual_steer() const override { return std::nullopt; }
    virtual std::optional<double> throttle() const override { return std::nullopt; }
  } m_state;

  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp) override;
  virtual const IVehicleState& update_and_get_current_state() override { return m_state; }

};

};
};
};

#endif // !CONTROLNODE_ESTIMATION_SIMESTIMATOR_SIMESTIMATOR_HPP