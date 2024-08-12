#include <control_node/estimation/istate_estimator.hpp>
#include <control_node/estimation/state_estimator_factory.hpp>

#include <control_node/estimation/noop_estimator/noop_estimator.hpp>

namespace control_node {
namespace estimation {

template <typename T>
static std::unique_ptr<IStateEstimator> create_state_estimator() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IStateEstimator>(*)()>> ESTIMATORS = {
  std::make_pair("Noop", create_state_estimator<noop::NoopEstimator>),
};

static constexpr ComponentFactory<IStateEstimator> FACTORY(ESTIMATORS);
const ComponentFactory<IStateEstimator>& get_factory() { return FACTORY; }

}; // namespace estimation
}; // namespace control_node