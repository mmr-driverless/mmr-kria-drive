#include <control_node/control/separate_long_lat/longitudinal_controller_factory.hpp>
#include <control_node/control/separate_long_lat/longitudinal/old_longitudinal/old_longitudinal.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

template <typename T>
static std::unique_ptr<ILongitudinalController> create_controller() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<ILongitudinalController>(*)()>> CONTROLLERS = {
    std::make_pair("OldLongitudinal", create_controller<longitudinal::old_longitudinal::OldLongitudinal>)

};

static constexpr ComponentFactory<ILongitudinalController> FACTORY(CONTROLLERS);
const ComponentFactory<ILongitudinalController>& get_longitudinal_controller_factory() { return FACTORY; }

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node