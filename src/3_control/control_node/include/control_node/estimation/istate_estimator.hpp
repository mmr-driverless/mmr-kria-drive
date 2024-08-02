#ifndef CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP

#include <control_node/estimation/vehicle_state.hpp>

namespace control_node {
namespace estimation {

struct IStateEstimator {
  virtual VehicleState update_and_get_current_state() = 0;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP