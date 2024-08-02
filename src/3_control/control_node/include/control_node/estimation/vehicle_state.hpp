#ifndef CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP

#include <Eigen/Dense>

namespace control_node {
namespace estimation {

struct VehicleState {
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  
  double yaw;
  double yaw_rate;
};

};
};

#endif // !CONTROLNODE_ESTIMATION_VEHICLESTATE_HPP