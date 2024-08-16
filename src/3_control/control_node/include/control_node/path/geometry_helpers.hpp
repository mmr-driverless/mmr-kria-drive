#ifndef GEOMETRY_HELPERS_HPP
#define GEOMETRY_HELPERS_HPP

#include <Eigen/Dense>
#include <utility>
#include <span>

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
 * @returns The instantaneous curvature.
 * @param fp The first derivative of the path with respect to space.
 * @param fpp The second derivative of the path with respect to space.
*/
inline double curv(Eigen::Vector2d fp, Eigen::Vector2d fpp) {
    return cross_2d(fp, fpp) / std::pow(fp.norm(), 3);
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

void curvature(bool is_loop, std::span<const double> ds, std::span<const Eigen::Vector2d> pts, std::span<double> k) {
    if (pts.size() < 3) {
        std::fill(k.begin(), k.end(), 0);
        return;
    }

    Eigen::Vector2d fp_first;
    Eigen::Vector2d fp_last;

    Eigen::Vector2d fp;
    Eigen::Vector2d fp_prev;
    Eigen::Vector2d fp_next = (pts[0] - pts[2]) / (ds[0] + ds[1]);
    
    // Compute the curvature at i = 0.
    {
        Eigen::Vector2d fpp;

        if (is_loop) {
            int pred_idx = pts.size() - 1;

            // Use central difference if we have a predecessor.
            fp_prev = (pts[0] - pts[pred_idx - 1]) / (ds[pred_idx - 1] + ds[pred_idx]);
            fp = (pts[1] - pts[pred_idx]) / (ds[pred_idx] + ds[0]);
            fpp = (fp_next - fp_prev) / (ds[pred_idx] + ds[0]);

            // Store these to later compute the last two curvatures!
            fp_first = fp;
            fp_last = fp_prev;
        } else {
            // Use forward difference if there is no predecessor.
            fp = (pts[1] - pts[0]) / ds[0];
            fpp = (fp_next - fp) / ds[0];
        }

        k[0] = curv(fp, fpp);
    }

    // Compute the middle part with central differences.
    for (int i = 1; i < pts.size() - 2; ++i) {
        fp_prev = fp;
        fp = fp_next;
        fp_next = (pts[i+2] - pts[i]) / (ds[i] + ds[i+1]);
        Eigen::Vector2d fpp = (fp_next - fp_prev) / (ds[i-1] + ds[i]);

        k[i] = curv(fp, fpp);
    }

    // Compute the curvature at i = n-2, n-1
    {
        fp_prev = fp;
        fp = fp_next;
        int curr_idx = pts.size() - 2;

        if (is_loop) {
            fp_next = fp_last;
        } else {
            // Use backwards difference if there is no successor to the next waypoint.
            fp_next = (pts[curr_idx + 1] - pts[curr_idx]) / ds[curr_idx];
        }

        // We can still use central difference for the second derivative (we're at the second to last element)
        Eigen::Vector2d fpp = (fp_next - fp_prev) / (ds[curr_idx] + ds[curr_idx - 1]);
        k[curr_idx] = curv(fp, fpp);

        // Now we are at the last waypoint
        fp_prev = fp;
        fp = fp_next;
        ++curr_idx;
        
        if (is_loop) {
            // We can use central differences because we have a successor
            fpp = (fp_first - fp_prev) / (ds[curr_idx - 1] + ds[curr_idx]);
        }
        else {
            // We need to use backwards difference, there is no successor
            fpp = (fp_prev - fp) / ds[curr_idx - 1];
        }

        k[curr_idx] = curv(fp, fpp);
    }
}

};


#endif // !GEOMETRY_HELPERS_HPP
