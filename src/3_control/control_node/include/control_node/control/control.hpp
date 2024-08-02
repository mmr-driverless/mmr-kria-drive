#ifndef CONTROLNODE_CONTROL_CONTROL_HPP
#define CONTROLNODE_CONTROL_CONTROL_HPP

namespace control_node {
namespace control {

class Control {
  double m_steer;
  double m_throttle;
  double m_brake;
  double m_clutch;
  int m_gear;
public:
  Control(double steer, double throttle, double brake, double clutch, int gear)
    : m_steer(steer), m_throttle(throttle), m_brake(brake), m_clutch(clutch), m_gear(gear) {}

  double steer() const { return m_steer; }
  double throttle() const { return m_throttle; }
  double brake() const { return m_brake; }
  double clutch() const { return m_clutch; }
  int gear() const { return m_gear; }
};

}; // namespace control
}; // namespace control_node

#endif // !CONTROLNODE_CONTROL_CONTROL_HPP