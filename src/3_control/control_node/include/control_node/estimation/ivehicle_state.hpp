#ifndef CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP

#include <Eigen/Dense>
#include "mmr_base/configuration.hpp"

namespace control_node {
namespace estimation {

struct IVehicleState {
  virtual AS::STATE as_state() const = 0;
  virtual int lap() const = 0;
  virtual Eigen::Vector2d position() const = 0;
  virtual Eigen::Vector2d velocity() const = 0;
  virtual double yaw() const = 0;
  virtual double yaw_rate() const = 0;
};

}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP