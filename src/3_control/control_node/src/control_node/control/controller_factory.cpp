#include <control_node/control/icontroller.hpp>
#include <control_node/control/controller_factory.hpp>

#include <control_node/control/pure_pursuit_2023/pure_pursuit_2023.hpp>
#include <control_node/control/lqr/lqr.hpp>
#include <control_node/control/inspection/inspection.hpp>
#include <control_node/control/long_step_response/long_step_response.hpp>

namespace control_node {
namespace control {

template <typename T>
static std::unique_ptr<IController> create_controller() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<IController>(*)()>> CONTROLLERS = {
  std::make_pair("PurePursuit2023", create_controller<pure_pursuit_2023::PurePursuit2023>),
  std::make_pair("LQR", create_controller<lqr::LQR>),
  std::make_pair("Inspection", create_controller<inspection::Inspection>),
  std::make_pair("LongStepResponse", create_controller<long_step_response::LongStepResponse>)
};

static constexpr ComponentFactory<IController> FACTORY(CONTROLLERS);
const ComponentFactory<IController>& get_factory() { return FACTORY; }

}; // namespace control
}; // namespace control_node