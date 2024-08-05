#ifndef CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP

#include <Eigen/Dense>

namespace control_node {
namespace estimation {

struct IVehicleState {
  virtual int lap() = 0;
  virtual Eigen::Vector2d position() = 0;
  virtual Eigen::Vector2d velocity() = 0;
  virtual double yaw() = 0;
  virtual double yaw_rate() = 0;
};

}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP