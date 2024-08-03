#ifndef CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP
#define CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

class PurePursuit2023 : public IController {
  const VehicleParameters& m_vp;
  double m_minLookForward;
  double m_steerGain;

public:
  PurePursuit2023(const Parameters& p, const VehicleParameters& vp);
  
  double minLookForward() const { return m_minLookForward; };

  virtual Control control(
    const estimation::VehicleState& state,
    const path::ReferencePath& reference_path,
    const std::optional<path::ReferencePath::PointRef>& vehicle_path_projection
  ) override;
};

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP