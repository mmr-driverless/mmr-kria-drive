#include <control_node/control/icontroller.hpp>
#include <control_node/control/controller_factory.hpp>

#include <control_node/control/separate_long_lat/separate_long_lat.hpp>
#include <control_node/control/inspection/inspection.hpp>
#include <control_node/control/long_step_response/long_step_response.hpp>

namespace control_node {
namespace control {

template <typename T>
static std::unique_ptr<IController> create_controller() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IController>(*)()>> CONTROLLERS = {
  std::make_pair("SeparateLongitudinalLateral", create_controller<separate_long_lat::SeparateLongitudinalLateralController>),
  std::make_pair("Inspection", create_controller<inspection::Inspection>),
  std::make_pair("LongStepResponse", create_controller<long_step_response::LongStepResponse>)
};

static constexpr ComponentFactory<IController> FACTORY(CONTROLLERS);
const ComponentFactory<IController>& get_factory() { return FACTORY; }

}; // namespace control
}; // namespace control_node