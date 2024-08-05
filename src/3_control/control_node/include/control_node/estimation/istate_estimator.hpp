#ifndef CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP
#define CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP

#include <control_node/estimation/ivehicle_state.hpp>

namespace control_node {
namespace estimation {

struct IStateEstimator {
  virtual const IVehicleState& update_and_get_current_state() = 0;
};


};
};

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATOR_HPP