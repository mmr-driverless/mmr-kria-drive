#ifndef CONTROLNODE_ESTIMATION_STATEESTIMATORFACTORY_HPP
#define CONTROLNODE_ESTIMATION_STATEESTIMATORFACTORY_HPP

#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/component_factory.hpp>

namespace control_node {
namespace estimation {

const ComponentFactory<IStateEstimator>& get_factory();

}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_STATEESTIMATORFACTORY_HPP