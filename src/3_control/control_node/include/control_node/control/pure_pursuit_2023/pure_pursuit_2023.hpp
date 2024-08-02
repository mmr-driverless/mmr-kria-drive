#ifndef CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP
#define CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP

#include <control_node/control/icontroller.hpp>
#include <control_node/vehicle_parameters.hpp>

namespace control_node {
namespace control {
namespace pure_pursuit_2023 {

class PurePursuit2023 : public IController {
  std::optional<path::ReferencePath::PointRef> m_old_path_ref;

  const VehicleParameters& m_vp;
  double m_minLookForward;
  double m_steerGain;

public:
  PurePursuit2023(const Parameters& p, const VehicleParameters& vp);
  
  constexpr virtual bool is_path_follower() override { return true; }
  virtual Control control(const estimation::VehicleState& state) override { assert(false); }
  virtual Control control(
    const estimation::VehicleState& state,
    const path::ReferencePath& reference_path
  ) override;
};

}; // namespace pure_pursuit_2023
}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_PUREPURSUIT2023_PUREPURSUIT2023_HPP