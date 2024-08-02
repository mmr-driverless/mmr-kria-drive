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

  VehicleState m_state;

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg);

public:
  // TODO: I hate the whole ass node reference just to create some subscriptions
  NoopEstimator(rclcpp::Node& node, const Parameters& p);
  virtual VehicleState update_and_get_current_state() override { return m_state; }
};

};
};
};

#endif // !CONTROLNODE_ESTIMATION_NOOPESTIMATOR_NOOPESTIMATOR_HPP