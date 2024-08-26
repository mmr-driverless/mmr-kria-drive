#ifndef STEP_RESPONSE_HPP
#define STEP_RESPONSE_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_node {
namespace control {
namespace step_response {

class StepResponse : public IController{

    const VehicleParameters* m_vp;
    double m_acc_target;
    int m_gear_target;
    double m_use_step;
    double m_min_throttle;

    public:
    // StepResponse();

    virtual void init(rclcpp::Node& node, const Parameters& p, const VehicleParameters& vp, rclcpp::Logger) override;

    virtual Control control(
        std::chrono::nanoseconds t,
        const estimation::IVehicleState& state,
        const path::ReferencePath& reference_path,
        const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
    ) override;

};

}; // namespace step_response
}; //namespace control
}; //namespace control_node

#endif