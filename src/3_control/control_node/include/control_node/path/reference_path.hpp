#ifndef REFERENCE_PATH_HPP
#define REFERENCE_PATH_HPP

#include <functional>
#include <optional>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <span>
#include <fstream>

#include <control_node/vehicle_parameters.hpp>
#include <control_node/path/geometry_helpers.hpp>
#include <control_node/path/non_uniform_first_order_filter.hpp>
#include <control_node/path/max_speed_eval.hpp>
#include <control_node/path/brake_velocity_saturation.hpp>

namespace control_node {
namespace path {

class ReferencePath {
public:
  class PointRef {
    // The previous waypoint index
    int prev_waypoint_idx;
    // Interpolation parameter between previous and next waypoint
    double t;

    PointRef(int idx, double t = 0) : prev_waypoint_idx(idx), t(t) {}
    friend ReferencePath;

  public:
    int get_waypoint_idx() const { return prev_waypoint_idx; }
    double get_t() const { return t; }
  };

  struct PathData {
    struct Metadata {
      bool is_target_speed_valid = false;
      bool is_curvature_valid = false;
      bool is_dist_to_next_valid = false;
    };
    struct Data {
      std::span<double> dist_to_next;
      std::span<double> curvature;
      std::span<double> target_speed;

      Data() = default;

      Data(
        std::span<double> dist_to_next,
        std::span<double> curvature,
        std::span<double> target_speed)
          : dist_to_next(dist_to_next), curvature(curvature), target_speed(target_speed)
      {}
    };

    Data data;
    Metadata metadata;

    size_t size() const {
      size_t n = data.dist_to_next.size();
      assert(
        n == data.curvature.size() && 
        n == data.target_speed.size() &&
        "All data fields must have the same size."
      );
      return n;
    }

    template <typename StreamT>
    static void csv_header(StreamT& o) {
      o << "dist_to_next, k, max_speed";
    }

    template <typename StreamT>
    void to_csv(StreamT& o, int i) {
      o << data.dist_to_next[i] << ", " << data.curvature[i] << ", " << data.target_speed[i];
    }

    PathData() {}

    PathData(std::span<double> dist_to_next, std::span<double> curvature, std::span<double> max_speed)
      : data(dist_to_next, curvature, max_speed)
    {}
  };

private:
  std::span<Eigen::Vector2d> m_waypoints;
  PathData m_data;

  bool m_is_closed;
  
  inline bool is_valid_reference(const PointRef& ref) const {
    return ref.prev_waypoint_idx >= 0 && ref.prev_waypoint_idx < n_waypoints() && ref.t >= 0 && ref.t <= 1;
  }

  int compute_index(int start, int offset) const {
    if(this->is_closed())
    {
      int idx = (start + offset) % n_waypoints();
      if(idx < 0)
      {
        return idx + n_waypoints();
      }
      return idx;
    }
    else
    {
      int idx = start + offset;
      if(idx < 0 || idx >= this->n_waypoints())
      {
        return -1;
      }
      return idx;
    }
  }

  struct SegmentRef {
    int start;
    int end;

    SegmentRef(int start, int end) : start(start), end(end) {}
  };

  std::optional<SegmentRef> compute_segment(int start) const {
    int end = compute_index(start, 1);
    if (end == -1)
      return {};
    return SegmentRef(start, end);
  }

  /** Compute the segment indices for the segment with relative start index.
  */
  std::optional<SegmentRef> compute_segment(int start, int offset) const {
    int start_idx = compute_index(start, offset);
    if (start_idx == -1)
      return {};
    return compute_segment(start_idx);
  }


  int m_waypoints_size;
  inline int n_waypoints() const { return m_waypoints_size; }

  int compute_waypoint_count(int start, int end) const {
    if (end < start) {
      return (n_waypoints() - start) + end;
    } else {
      return end - start;
    }
  }

public:

  ReferencePath(std::span<Eigen::Vector2d> waypoints, PathData data, bool is_closed)
    : m_waypoints(waypoints), m_data(data), m_is_closed(is_closed)
  {
    assert (waypoints.size() == data.size() && "The waypoints and data views must have the same size.");
    m_waypoints_size = (int)waypoints.size();
  }

  ReferencePath()
    : m_waypoints(),
      m_data(),
      m_is_closed(false),
      m_waypoints_size(0)
  {}

  std::optional<PointRef> start() const {
    if (n_waypoints() > 0)
      return PointRef(0);
    else
      return {};
  }

  struct ProjectionResult {
    PointRef closest_point;
    PointRef first_checked_waypoint;
    PointRef last_checked_waypoint;

    ProjectionResult(PointRef closest, PointRef first, PointRef last)
      : closest_point(closest),
        first_checked_waypoint(first),
        last_checked_waypoint(last) { }

    explicit ProjectionResult(PointRef single)
      : ProjectionResult(single, single, single) { }
  };

  /**
   * Project the vehicle state onto the path.
   * @param position The current vehicle position [m, m]
   * @param last_ref The last known path point.
   */
  std::optional<ProjectionResult> project_vehicle(Eigen::Vector2d position, std::optional<PointRef> last_ref, double threshold_squared) const {
    /*
      Find the closest point on the path.

      Both to optimize this and for Skidpad to work at all (there're intersections),
      we track the progress of the vehicle by taking into account the latest closest point.
      
      We assume that the vehicle remains on track (keeps a distance < threshold from the centerline,
      where the threshold distance is most likely the track width). With this assumption, we know that
      we should search for the closest point in the first region of track with distance < threshold from the vehicle,
      starting the search for the region from the last reference.

      If we don't find such a region, it means that the vehicle is off course. In that case, we
      take the globally closest point to the vehicle, which is the optimal solution for all events but Skidpad.
    */
   
    if (n_waypoints() <= 0)
      return {};
    if (n_waypoints() == 1)
      return ProjectionResult(PointRef(0));

    
    bool inside_thresh_region = false;

    assert(!last_ref.has_value() || is_valid_reference(*last_ref) && "last_ref must either be empty or a valid reference!");
    const int start_waypoint_idx = last_ref.has_value()? last_ref->prev_waypoint_idx : 0;

    // At this point it is guaranteed that we find a closest point. Keep it optional so that we can assert this assumption later.
    std::optional<PointRef> closest_point;
    int last_waypoint_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    // Iterate over all waypoints starting from (and including) that of the last reference
    for (int rel_i = 0; rel_i < n_waypoints(); ++rel_i) {
      int start_idx = compute_index(start_waypoint_idx, rel_i);
      if (start_idx < 0)
        break; // end of path reached
      
      last_waypoint_idx = start_idx;

      auto start = m_waypoints[start_idx];

      // Compute the locally closest point and its distance.
      // - either the current waypoint or the closest point on the segment which starts at this waypoint.
      PointRef locally_closest_pt(start_idx);
      double d;
      if (auto seg = compute_segment(start_idx)) {
        // This waypoint is the start of a segment - compute the point-segment distance.
        auto end = m_waypoints[seg->end];

        auto [t, pt] = geometry_helpers::pt_segment_projection(position, start, end);
        d = (pt - position).squaredNorm();
        
        // Store the projection parameter.
        locally_closest_pt.t = t;
      } else {
        // This is the last waypoint in the path.
        d = (position - start).squaredNorm();
      }

      // Perform the threshold logic.
      // If we already entered the threshold region,
      if (inside_thresh_region) {
        // If we're now leaving it, then stop traversing the path
        if (d > threshold_squared)
          break;
      } else {
        // Otherwise, if we're now entering it
        if (d <= threshold_squared)
          inside_thresh_region = true;
      }

      // Evaluate whether this is the new closest point.
      if (d <= min_dist) {
        min_dist = d;
        closest_point = locally_closest_pt;
      }
    }

    assert(closest_point.has_value());
    assert(last_waypoint_idx >= 0);
    assert(is_valid_reference(*closest_point));

    return ProjectionResult(
      *closest_point,
      PointRef(start_waypoint_idx, 0.0),
      PointRef(last_waypoint_idx, 0.0)
    );
  }

  /**
   * Advance a path point (lookforward)
   * @param at The path point to advance
   * @param delta_s How much to advance the point [m]
   */
  PointRef advance_point(const PointRef& at, double delta_s) const {
    assert(is_valid_reference(at) && "at must be a valid reference.");
    assert(delta_s >= 0 && "'delta_s' must be nonnegative.");
    
    PointRef cur = at;

    // Traverse the waypoints to consume delta_s of space
    for (int rel_i = 0; rel_i < n_waypoints(); ++rel_i) {
      auto seg = compute_segment(at.prev_waypoint_idx, rel_i);
      if (!seg.has_value())
        break;

      auto start = get_position(PointRef(seg->start));
      auto end = get_position(PointRef(seg->end));
      
      // Compute the distance between the current position and the next waypoint
      double seg_len = (end - start).norm();
      if (seg_len > 0) {
        double ds = (1 - cur.t) * seg_len;

        // If we would consume more space than needed by advancing to the next waypoint
        if (ds > delta_s) {
          /* Then the search ends here.
              Add the fraction of remaining space to the parameter part of PointRef.
              Addition is required only for the start reference (nonzero start t),
              while for the rest (if we've advanced by even just one waypoint) this is equivalent to setting it directly (t is zero).
          */
          cur.t += (delta_s / seg_len);
          break;
        }

        delta_s -= ds;
      }
      
      // Advance to the next waypoint.
      cur = PointRef(seg->end);
    }

    return cur;
  }

  PointRef trace_back_point(const PointRef& at, double delta_s) const {
    assert(is_valid_reference(at) && "'at' must be a valid reference");
    assert(delta_s >= 0 && "'delta_s' must be nonnegative.");

    // Retreat to the segment start waypoint, if we're in the middle of one
    if (at.t > 0) {
      if (auto seg = compute_segment(at.prev_waypoint_idx)) {
        double segment_length = (m_waypoints[seg->end] - m_waypoints[seg->start]).norm();

        if (segment_length > 0) {
          double dist_from_start = segment_length * at.t;

          // If we would not deplete delta_s
          if (dist_from_start < delta_s)
            delta_s -= dist_from_start;
          else
            return PointRef(at.prev_waypoint_idx, at.t - delta_s / segment_length);
        }
      }
    }

    // Iterate over each segment (backwards) to consume delta_s of space
    int end_i = at.prev_waypoint_idx;
    while (delta_s > 0) {
      int start_i = compute_index(end_i, -1);

      // If we reached the end of the path
      if (start_i < 0)
        return PointRef(end_i);
      
      // Compute this segment length
      double segment_length = (m_waypoints[end_i] - m_waypoints[start_i]).norm();
      if (segment_length > 0) {
        // If we would not deplete delta_s
        if (segment_length < delta_s)
          delta_s -= segment_length;
        else
          return PointRef(start_i, 1 - delta_s / segment_length);
      }
      end_i = start_i;
    }

    return PointRef(end_i);
  }

  /**
   * Get the path position at the specified reference.
   * @param at The location.
   */
  Eigen::Vector2d get_position(const PointRef& at) const {
    // prev_waypoint_idx must be a valid waypoint
    assert(at.prev_waypoint_idx >= 0 && at.prev_waypoint_idx < n_waypoints());

    std::optional<SegmentRef> seg = compute_segment(at.prev_waypoint_idx);
    if (!seg.has_value()) // There's no segment at that index (that waypoint doesn't have a successor)
      return m_waypoints[at.prev_waypoint_idx]; // return the waypoint itself

    // Interpolate the waypoints for this segment
    return m_waypoints[seg->start] + at.t * (m_waypoints[seg->end] - m_waypoints[seg->start]);
  }

  std::array<std::span<Eigen::Vector2d>, 2> get_subpath(const PointRef& start, const PointRef& end) const {
    int start_idx = start.prev_waypoint_idx;
    int end_idx = end.prev_waypoint_idx;

    // Supporting looping around (without copies) in any other way would require a custom iterator. Just return two spans.
    if (start_idx <= end_idx)
      return {
        m_waypoints.subspan(start_idx, end_idx - start_idx),
        std::span<Eigen::Vector2d>()
      };
    else 
      return {
        m_waypoints.subspan(start_idx, n_waypoints() - start_idx),
        m_waypoints.subspan(0, end_idx)
      };
  }

  std::optional<double> get_target_speed(const PointRef& at) const {
    if (n_waypoints() <= 0 || !m_data.metadata.is_target_speed_valid)
      return std::nullopt;

    assert(is_valid_reference(at) && "at must be a valid reference.");

    double speed = m_data.data.target_speed[at.prev_waypoint_idx];

    int succ_idx = compute_index(at.prev_waypoint_idx, 1);
    if (succ_idx < 0)
      return speed;

    return speed + (m_data.data.target_speed[succ_idx] - speed) * at.t;
  }

  std::optional<double> get_curvature(const PointRef& at) const {
    if (n_waypoints() <= 0 || !m_data.metadata.is_curvature_valid)
      return std::nullopt;

    assert(is_valid_reference(at) && "at must be a valid reference.");

    double k = m_data.data.curvature[at.prev_waypoint_idx];

    int succ_idx = compute_index(at.prev_waypoint_idx, 1);
    if (succ_idx < 0)
      return k;

    return k + (m_data.data.curvature[succ_idx] - k) * at.t;
  }

  void compute_data(const VehicleParameters& vp) {
    if (!m_data.metadata.is_dist_to_next_valid) {
      // Compute, for each waypoint, the length of the segment that connects it to the next waypoint
      for (int i = 0; i < n_waypoints(); ++i) {
        int next_i = compute_index(i, 1);
        m_data.data.dist_to_next[i] = next_i >= 0? (m_waypoints[next_i] - m_waypoints[i]).norm() : NAN;
      }

      m_data.metadata.is_dist_to_next_valid = true;
    }

    if (!m_data.metadata.is_curvature_valid) {
      geometry_helpers::curvature(is_closed(), m_data.data.dist_to_next, m_waypoints, m_data.data.curvature);

      // For simplicity, we assume the start and end are straights.
      m_data.data.curvature.front() = 0;
      m_data.data.curvature.back() = 0;

      /*
      Filter the path curvature.
      We have a nonuniformly sampled signal k(s), or the curvature at a certain arc length.
      The signal is extremely noisy, as we're computing the instantaneous, local curvature at each waypoint.
      This results in a lot of high-frequency noise.
      
      Our filter:
      - must have zero-phase
          ^ we don't want the speed profile to be late
      - must be quick
          ^ we don't want to resample the signal (if you want to try, good luck with aliasing)
      - must be a low pass filter
          ^ we tried a simple weighted moving average, but unfortunately it doesn't cut it

      Due to the non-uniform sampling we use a standard causal continuous IIR filter with a bilinear approximation
      (see the NonUniformBilinearApproxIIRFilter class). This is kind of expensive, but it's hard to get wrong.

      Obviously, being a causal filter it does not have zero phase, 
      so we perform one forward pass and a backwards one, just like "filtfilt" from MATLAB.

      As a filter, we chose a Butterworth filter. 3rd is the highest order that takes a reasonable computational time (see the NonUniformBi... whatever).
      Cascading two of them results in a reasonable computation cost and a good response.

      Achieving a similar result with a single 4th order filter takes double the time (ouch)!

      The filter state-space matrices were obtained with:
      [A,B,C,D] = butter(3, 0.6, 's')

      Please note that you need to design a CONTINUOUS time (ANALOG) filter!
      */

      const double Wn = vp.curv_cutoff_radps();
      const NonUniformBilinearApproxIIRFilter<3> FILTER_PROTOTYPE(
        0, Eigen::Matrix<double, 3, 1>::Zero(), // We assume that the start and end are straights!!
        Eigen::Matrix<double, 3, 3> {
          { -Wn, 0, 0 },
          { Wn, -Wn, -Wn },
          { 0, Wn, 0 }
        },
        Eigen::Matrix<double, 3, 1> {
          { Wn },
          { 0 },
          { 0 }
        },
        Eigen::Matrix<double, 1, 3> {
          { 0, 0, 1 }
        },
        0
      );

      {
        auto fil1 = FILTER_PROTOTYPE;
        auto fil2 = FILTER_PROTOTYPE;
        for (int i = 1; i < n_waypoints(); ++i) {
          double ds = m_data.data.dist_to_next[i-1];
          m_data.data.curvature[i] = fil2(fil1(m_data.data.curvature[i], ds), ds);
        }
      }

      {
        auto fil1 = FILTER_PROTOTYPE;
        auto fil2 = FILTER_PROTOTYPE;
        for (int i = n_waypoints() - 2; i >= 0; --i) {
          double ds = m_data.data.dist_to_next[i];
          m_data.data.curvature[i] = fil2(fil1(m_data.data.curvature[i], ds), ds);
        }
      }

      m_data.metadata.is_curvature_valid = true;
    }

    if (!m_data.metadata.is_target_speed_valid) {
      // Compute the maximum pure-cornering velocity given the curvature
      for (int i = 0; i < n_waypoints(); ++i)
        m_data.data.target_speed[i] = speed::evaluateMaxSpeed(m_data.data.curvature[i]);
      
      // Saturate with brake potential
      braking::saturate_velocity_with_brake_potential(m_data.data.dist_to_next, m_data.data.target_speed, vp.brake_potential_deceleration(), m_is_closed);
      
      m_data.metadata.is_target_speed_valid = true;
    }
  }

  void dump(const std::string& path) {
    Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");
    
    std::ofstream f(path);
    f << "x, y, ";
    PathData::csv_header(f);
    f << "\n";

    for (int i = 0; i < n_waypoints(); ++i) {
      f << m_waypoints[i].format(CSVFormat) << ", ";
      m_data.to_csv(f, i);
      f << "\n";
    }
  }

  bool is_closed() const { return m_is_closed; } 
};

}; // namespace path
}; // namespace control_node

#endif 
