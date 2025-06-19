#include <control_node/control/icontroller.hpp>
#include <control_node/control/separate_long_lat/lateral_controller_factory.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

template <typename T>
static std::unique_ptr<IController> create_controller() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<ILateralController>(*)()>> CONTROLLERS = {
  
};

static constexpr ComponentFactory<ILateralController> FACTORY(CONTROLLERS);
const ComponentFactory<ILateralController>& get_factory() { return FACTORY; }

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node