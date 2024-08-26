/*

See this for reference:

L. Fesquet, B. Bidégaray-Fesquet,
IIR digital filtering of non-uniformly sampled signals via state representation,
Signal Processing,
Volume 90, Issue 10,
2010,
Pages 2811-2821,
ISSN 0165-1684,
https://doi.org/10.1016/j.sigpro.2010.03.030.
(https://www.sciencedirect.com/science/article/pii/S0165168410001349)

*/


#ifndef CONTROLNODE_PATH_NONUNIFORMFIRSTORDERFILTER_HPP
#define CONTROLNODE_PATH_NONUNIFORMFIRSTORDERFILTER_HPP

#include <cassert>
#include <Eigen/Dense>

namespace control_node {
namespace path {

template <int Order>
class NonUniformBilinearApproxIIRFilter {
  static_assert(Order <= 3 && "No. Don't. Trust me. Matrix inverses blow up with such a large order.");

  // The state variable at the previous iteration
  Eigen::Matrix<double, Order, 1> x_nm1;

  // The filter input at the previous iteration
  double u_nm1;

  Eigen::Matrix<double, Order, Order> A;
  Eigen::Matrix<double, Order, 1> B;
  Eigen::Matrix<double, 1, Order> C;
  Eigen::Matrix<double, 1, 1> D;

public:
  /**
    @param A,B,C,D Continous-time filter state space matrices.
    @param u0 The initial input of the filter.
    @param x0 The initial internal state of the filter.
  */
  NonUniformBilinearApproxIIRFilter(
    double u0,
    Eigen::Matrix<double, Order, 1> x0,
    Eigen::Matrix<double, Order, Order> A,
    Eigen::Matrix<double, Order, 1> B,
    Eigen::Matrix<double, 1, Order> C,
    double D)
    : x_nm1(x0),
      u_nm1(u0),
      A(A), B(B), C(C), D(D)
  { }

  /**
    @param x_n The filter input at the current step.
    @param step_length The delta time or space since the last sample
   */
  double operator()(double u_n, double step_length) {
    assert(step_length >= 0 && "Step length must be nonnegative.");

    // See chapter 2.4 of the paper. This is the bilinear approximation.

    // Common subexpressions
    Eigen::Matrix<double, Order, Order> M = (step_length / 2.0) * A;
    Eigen::Matrix<double, Order, Order> Id_m_M_inv = (Eigen::Matrix<double, Order, Order>::Identity() - M).inverse();
    Eigen::Matrix<double, Order, Order> Id_p_M = Eigen::Matrix<double, Order, Order>::Identity() + M;

    Eigen::Matrix<double, Order, Order> phi = Id_m_M_inv * Id_p_M;
    Eigen::Matrix<double, Order, 1> lambda = Id_m_M_inv * step_length * B;

    // Compute x at the current step (y at n)
    Eigen::Matrix<double, Order, 1> x_n = phi * x_nm1 + lambda * (u_n + u_nm1) / 2;

    // Store the values for the next step
    x_nm1 = x_n;
    u_nm1 = u_n;

    Eigen::Matrix<double, 1, 1> ans = C * x_n + D * u_n;
    return ans(0);
  }
};

}; // namespace path
}; // namespace control_node

#endif // !CONTROLNODE_PATH_NONUNIFORMFIRSTORDERFILTER_HPP