#ifndef CONTROLNODE_PATH_MAXSPEEDEVAL_HPP
#define CONTROLNODE_PATH_MAXSPEEDEVAL_HPP

#include <cmath>
#include <array>

namespace control_node {
namespace path {
namespace speed {

// polyeval_targetVelocity
inline float evaluateMaxSpeed(float curvature)
{
  curvature = std::max<float>(std::abs(curvature), 0.01f);
  float radius = 1 / curvature;

  constexpr std::array<float, 8> velocity_coeffs = {
    0.5051f, 1.063f, -0.02495f, 0.0003832f, -3.251e-6f, 1.494e-8f, -3.511e-11f, 3.32e-14f
  };

  float result = 0.0f;
  for (int i = 0; i < (int)velocity_coeffs.size(); i++)
    result += velocity_coeffs[i] * std::pow(radius, i);

  return result;
}

}; // namespace speed
}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_MAXSPEEDEVAL_HPP