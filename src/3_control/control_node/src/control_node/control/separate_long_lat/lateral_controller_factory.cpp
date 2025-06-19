
#include <control_node/control/separate_long_lat/lateral_controller_factory.hpp>
#include <control_node/control/separate_long_lat/lateral/lqr/lqr.hpp>
#include <control_node/control/separate_long_lat/lateral/pure_pursuit/pure_pursuit.hpp>

namespace control_node {
namespace control {
namespace separate_long_lat {

template <typename T>
static std::unique_ptr<ILateralController> create_controller() { return std::make_unique<T>(); }

static constexpr std::initializer_list<std::pair<const char*, std::unique_ptr<ILateralController>(*)()>> CONTROLLERS = {
    std::make_pair("LQR", create_controller<lateral::lqr::LQR>),
    std::make_pair("PurePursuit", create_controller<lateral::pure_pursuit::PurePursuit>)
};

static constexpr ComponentFactory<ILateralController> FACTORY(CONTROLLERS);
const ComponentFactory<ILateralController>& get_factory() { return FACTORY; }

}; // namespace separate_long_lat
}; // namespace control
}; // namespace control_node