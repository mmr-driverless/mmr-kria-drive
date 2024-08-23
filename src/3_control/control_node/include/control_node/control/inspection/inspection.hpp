#ifndef CONTROLNODE_CONTROL_INSPECTION_INSPECTION_HPP
#define CONTROLNODE_CONTROL_INSPECTION_INSPECTION_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_node {
namespace control {
namespace inspection {

class Inspection : public IController {
    const VehicleParameters* m_vp;
    std::optional<std::chrono::nanoseconds> m_start_t;
    double m_frequency;
    double m_amplitude;
    double m_throttle;
    int m_gear;
    Control::LaunchControl m_lc;
    
public:
    virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, viz::VizManager&, rclcpp::Logger) override;

    virtual Control control(
        std::chrono::nanoseconds t,
        const estimation::IVehicleState& state,
        const path::ReferencePath& reference_path,
        const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection,
        int lap
    ) override;
};

}; // namespace inspection
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_INSPECTION_INSPECTION_HPP