#ifndef CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <control_node/estimation/ivehicle_state.hpp>
#include <control_node/parameters.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace estimation {

struct IStateEstimator {
  virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp) = 0;
  virtual const IVehicleState& update_and_get_current_state() = 0;
  virtual ~IStateEstimator() = default;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP