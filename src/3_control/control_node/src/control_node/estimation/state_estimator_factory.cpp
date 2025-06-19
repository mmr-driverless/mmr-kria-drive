#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/estimation/state_estimator_factory.hpp>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>
#include <control_node/estimation/inspection/inspection_estimator.hpp>
#include <control_node/estimation/sim_estimator/sim_estimator.hpp>

namespace control_node {
namespace estimation {

template <typename T>
static std::unique_ptr<IStateEstimator> create_state_estimator() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IStateEstimator>(*)()>> ESTIMATORS = {
  std::make_pair("Noop", create_state_estimator<noop::NoopEstimator>),
  std::make_pair("Sim", create_state_estimator<sim::SimEstimator>),
  std::make_pair("Inspection", create_state_estimator<inspection::InspectionEstimator>)
};

static constexpr ComponentFactory<IStateEstimator> FACTORY(ESTIMATORS);
const ComponentFactory<IStateEstimator>& get_factory() { return FACTORY; }

}; // namespace estimation
}; // namespace control_node