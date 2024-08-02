#ifndef CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP

#include <Eigen/Dense>

namespace control_node {
namespace estimation {

class VehicleState {
  Eigen::Vector2d m_position;
  Eigen::Vector2d m_velocity;
  
  double m_yaw;
  double m_yaw_rate;

public:
  inline Eigen::Vector2d position() const { return m_position; }
  inline Eigen::Vector2d velocity() const { return m_velocity; }
  inline double yaw() const { return m_yaw; }
  inline double yaw_rate() const { return m_yaw_rate; }
};

};
};

#endif // !CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP