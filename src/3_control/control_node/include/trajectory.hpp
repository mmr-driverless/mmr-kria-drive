#include <optional>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <span>

using PointT = Eigen::Vector2d;
using DataT = Eigen::Vector3d;

static inline std::pair<double, PointT> pt_segment_projection(PointT pt, PointT start, PointT end) {
  auto delta = (end - start);
  double t = std::clamp(delta.dot(pt), 0.0, 1.0);
  return { t, start + delta * t };
}

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

  struct PointData {
    double yaw;
    double curvature;
    double max_speed;

    friend ReferencePath;
  private:
    PointData(DataT data) : yaw(data(0)), curvature(data(1)), max_speed(data(2)) {}
  };

private:
  std::span<PointT> m_waypoints;
  std::span<DataT> m_data;

  bool m_is_closed;
  
  int compute_index(int start, int offset) const {
    int idx = start + offset;

    // If the trajectory is closed, we can loop around
    if (is_closed())
      return idx % n_waypoints();

    // Otherwise, if the trajectory is open, we don't loop around
    else if (idx < n_waypoints())
        return idx;

    // Invalid index
    return -1;
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

public:

  // Passing spans around because it must be readily apparent that ReferencePath is a stateless object.
  ReferencePath(std::span<PointT> waypoints, std::span<DataT> data) : m_waypoints(waypoints), m_data(data) {
    assert (waypoints.size() == data.size() && "The waypoints and data views must have the same size.");
    m_waypoints_size = (int)waypoints.size();
  }

  ReferencePath() { }

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
  std::optional<ProjectionResult> project_vehicle(PointT position, PointRef last_ref) const {
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

    assert(last_ref.prev_waypoint_idx >= 0 && last_ref.prev_waypoint_idx < n_waypoints() && "last_window_start must be a valid reference.");

    constexpr double threshold = 10.0;
    bool inside_thresh_region = false;

    const int start_waypoint_idx = last_ref.prev_waypoint_idx;

    // At this point it is guaranteed that we find a closest point. Keep it optional so that we can assert this assumption later.
    std::optional<PointRef> closest_point;
    int last_waypoint_idx = -1;
    double min_dist = std::numeric_limits<double>::max();

    // Iterate over all waypoints starting from (and including) that of the last reference
    for (int rel_i = 0; rel_i < n_waypoints(); ++rel_i) {
      int start_idx = compute_index(start_waypoint_idx, rel_i);
      if (start_idx < 0)
        break; // end of path reached
      
      auto start = m_waypoints[start_idx];
      last_waypoint_idx = start_idx;

      // Compute the locally closest point and its distance.
      // - either the current waypoint or the closest point on the segment which starts at this waypoint.
      PointRef locally_closest_pt(start_idx);
      double d;

      int end_idx = compute_index(start_idx, 1);
      if (end_idx < 0) {
        // This is the last waypoint in the path.
        d = (position - start).squaredNorm();
      } else {
        // This waypoint is the start of a segment - compute the point-segment distance.
        auto end = m_waypoints[end_idx];

        auto [t, pt] = pt_segment_projection(position, start, end);
        d = (pt - position).squaredNorm();
        
        // Store the projection parameter.
        locally_closest_pt.t = t;
      }

      // Evaluate whether this is the new closest point.
      if (d <= min_dist) {
        min_dist = d;
        closest_point = locally_closest_pt;
      }

      // Perform the threshold logic.
      // If we already entered the threshold region,
      if (inside_thresh_region) {
        // If we're now leaving it, then stop traversing the path
        if (d > threshold) {
          break;
        }
      } else {
        // Otherwise, if we're now entering it
        if (d <= threshold)
          inside_thresh_region = true;
      }
    }

    assert(closest_point.has_value());
    assert(last_waypoint_idx >= 0);

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
    assert(at.prev_waypoint_idx >= 0 && at.prev_waypoint_idx < n_waypoints() && "'at' must be a valid reference");
    assert(delta_s >= 0 && "'delta_s' must be nonnegative.");
    
    PointRef cur = at;

    // Traverse the waypoints to consume delta_s of space
    for (int rel_i = 0; rel_i < n_waypoints(); ++rel_i) {
      auto seg = compute_segment(at.prev_waypoint_idx, rel_i);
      if (!seg.has_value())
        break;

      // Compute the distance between the current position and the next waypoint
      double ds = (get_position(PointRef(seg->end)) - get_position(cur)).norm();

      // If we would consume more space than needed by advancing to the next waypoint
      if (ds > delta_s) {
        /* Then the search ends here.
            Add the fraction of remaining space to the parameter part of PointRef.
            Addition is required only for the start reference (nonzero start t),
            while for the rest (if we've advanced by even just one waypoint) this is equivalent to setting it directly (t is zero).
        */
        cur.t += (delta_s / ds);
        break;
      }
      
      // Advance to the next waypoint.
      delta_s -= ds;
      cur = PointRef(seg->end);
    }

    return cur;
  }

  /**
   * Get the path position at the specified reference.
   * @param at The location.
   */
  PointT get_position(const PointRef& at) const {
    // prev_waypoint_idx must be a valid waypoint
    assert(at.prev_waypoint_idx >= 0 && at.prev_waypoint_idx < n_waypoints());

    std::optional<SegmentRef> seg = compute_segment(at.prev_waypoint_idx);
    if (!seg.has_value()) // There's no segment at that index (that waypoint doesn't have a successor)
      return m_waypoints[at.prev_waypoint_idx]; // return the waypoint itself

    // Interpolate the waypoints for this segment
    return m_waypoints[seg->start] + at.t * (m_waypoints[seg->end] - m_waypoints[seg->start]);
  }

  /**
   * Get the path data at the specified reference.
   * @param at The location.
   */
  PointData get_data(const PointRef& at) const {
    // prev_waypoint_idx must be a valid waypoint
    assert(at.prev_waypoint_idx >= 0 && at.prev_waypoint_idx < n_waypoints());

    std::optional<SegmentRef> seg = compute_segment(at.prev_waypoint_idx);
    if (!seg.has_value()) // There's no segment at that index (that waypoint doesn't have a successor)
      return PointData(m_data[at.prev_waypoint_idx]); // return the waypoint data itself

    // Interpolate the data for this segment
    return PointData(
      m_data[seg->start] + at.t * (m_data[seg->end] - m_data[seg->start])
    );
  }

  bool is_closed() const { return m_is_closed; } 
};