#ifndef CONTROLNODE_VEHICLEPARAMETERS_HPP
#define CONTROLNODE_VEHICLEPARAMETERS_HPP

#include <control_node/parameters.hpp>

namespace control_node {

class VehicleParameters {
  double m_wheelbase; // Distance between front and rear axles [m]
  double m_lr; // Distance between CoM and rear axle [m]
  double m_max_steering_angle; // Maximum angle (duh) of the steered wheel in the bicycle model  [rad]

public:
  VehicleParameters(const Parameters& p)
    : m_wheelbase(p.get<double>("wheelbase")),
      m_lr(p.get<double>("lr")),
      m_max_steering_angle(p.get<double>("max_steering_angle"))
  {}

  double max_steering_angle() const { return m_max_steering_angle; }
  double wheelbase() const { return m_wheelbase; }
  double lr() const { return m_lr; }
};

};

#endif // !CONTROLNODE_VEHICLEPARAMETERS_HPP