#ifndef CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP

#include <Eigen/Dense>

namespace control_node {
namespace estimation {

struct IVehicleState {
  virtual int lap() const = 0;
  virtual Eigen::Vector2d position() const = 0;
  virtual Eigen::Vector2d velocity() const = 0;
  virtual double yaw() const = 0;
  virtual double yaw_rate() const = 0;
};

}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP