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
  struct VehicleState : public IVehicleState {
    Eigen::Vector2d m_position;
    double m_yaw;
    
    friend NoopEstimator;
  private:
    virtual AS::STATE as_state() const override { return AS::STATE::DRIVING; }
    virtual int lap() const override { return 0; }
    virtual Eigen::Vector2d position() const override { return m_position; }
    virtual Eigen::Vector2d velocity() const override { return Eigen::Vector2d::Zero(); }
    virtual double yaw() const override { return m_yaw; }
    virtual double yaw_rate() const override { return 0; }
  } m_state;

  NoopEstimator(rclcpp::Node& node, const Parameters& p);
  virtual const IVehicleState& update_and_get_current_state() override { return m_state; }

};

};
};
};

#endif // !CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP