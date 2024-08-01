#ifndef GEOMETRY_HELPERS_HPP
#define GEOMETRY_HELPERS_HPP

#include <Eigen/Dense>
#include <utility>

namespace geometry_helpers {
  
/**
 * @returns The parameter and point resulting from projecting pt onto the segment starting at start and ending at end.
 */
static inline std::pair<double, Eigen::Vector2d> pt_segment_projection(Eigen::Vector2d pt, Eigen::Vector2d start, Eigen::Vector2d end) {
  auto delta = (end - start);
  double t = (pt - start).dot(delta) / delta.squaredNorm();
  t = std::clamp(t, 0.0, 1.0);
  return { t, start + t * delta };
}

/**
 * @returns The magnitude of the cross product between two 2D vectors (3D with Z=0).
 *          Equivalently, the determinant of the block matrix [a, b], or the signed area of the parallelogram created by the two vectors.
 */
inline double cross_2d(Eigen::Vector2d a, Eigen::Vector2d b) {
  return a.x() * b.y() - a.y() * b.x();
}

/**
 * @returns The curvature of the circle passing through the three (ordered) points, with sign following the right hand rule (>0 for positive angular velocity).
 */
inline double menger_curvature(Eigen::Vector2d prev, Eigen::Vector2d curr, Eigen::Vector2d next) {
  // Signed area of triangle
  double A = cross_2d(curr - prev, next - curr) / 2;
  
  return (4*A) / std::sqrt(
    (curr - prev).squaredNorm() *
    (next - curr).squaredNorm() *
    (prev - next).squaredNorm()
  );
}

};


#endif // !GEOMETRY_HELPERS_HPP