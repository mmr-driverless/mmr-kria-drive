#ifndef CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP
#define CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP

#include <Eigen/Dense>
#include <optional>

namespace control_node {
namespace estimation {

struct IVehicleState {
  virtual std::optional<Eigen::Vector2d> position() const = 0;
  virtual std::optional<double> yaw() const = 0;
  virtual std::optional<double> yaw_rate() const = 0;
  virtual std::optional<double> speed() const = 0;
  virtual std::optional<double> vy() const = 0;
  virtual std::optional<int> rpm() const = 0;
  virtual std::optional<bool> lc_is_active() const = 0;
  virtual std::optional<bool> clutch_is_engaged() const = 0;
  virtual std::optional<int> gear() const = 0;
  virtual std::optional<bool> res_bag() const = 0;
  virtual std::optional<bool> res_go() const = 0;
  virtual std::optional<double> actual_steer() const = 0;
  virtual std::optional<double> throttle() const = 0;
};

}; // namespace estimation
}; // namespace control_node

#endif // !CONTROLNODE_ESTIMATION_IVEHICLESTATE_HPP