#include <optional>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <span>

using WaypointT = Eigen::Vector2d;
using DataT = Eigen::Vector3d;

static inline std::pair<double, WaypointT> pt_segment_projection(WaypointT pt, WaypointT start, WaypointT end) {
  auto delta = (end - start);
  double t = std::clamp(delta.dot(pt), 0.0, 1.0);
  return { t, start + delta * t };
}

class ReferencePath {
public:
  class WaypointRef {
    int idx;
    WaypointRef(int idx) : idx(idx) {}
    friend ReferencePath;
  public:
    WaypointRef() : idx(0) {}
    int get() const { return idx; }
  };

  class PointRef {
    // The previous waypoint index
    int prev_waypoint_idx;
    // Interpolation parameter between previous and next waypoint
    double t;

    PointRef(int idx, double t = 0) : prev_waypoint_idx(idx), t(t) {}
    friend ReferencePath;

  public:
    int get() const { return prev_waypoint_idx; }
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
  std::span<WaypointT> m_waypoints;
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
  ReferencePath(std::span<WaypointT> waypoints, std::span<DataT> data) : m_waypoints(waypoints), m_data(data) {
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
    PointRef track_point;
    WaypointRef window_start;

    friend ReferencePath;
  private:
    ProjectionResult(PointRef track_point, WaypointRef window_start) : track_point(track_point), window_start(window_start) {}
  };

  /**
   * Project the vehicle state onto the path.
   * @param position The current vehicle position [m, m]
   * @param last_ref The last known path point.
   */
  std::optional<ProjectionResult> project_vehicle(WaypointT position, WaypointRef last_window_start) const {
    if (n_waypoints() <= 0)
      return {};
    if (n_waypoints() == 1)
      return ProjectionResult(0, 0);

    assert(last_window_start.idx >= 0 && last_window_start.idx < n_waypoints() && "last_window_start must be a valid reference.");

    constexpr double threshold = 10.0;

    std::optional<WaypointRef> new_window_start;
    double min_dist = std::numeric_limits<double>::max();
    PointRef closest_point(last_window_start.idx);

    for (int rel_i = 0; rel_i < n_waypoints(); ++rel_i) {
      int start_idx = compute_index(last_window_start.idx, rel_i);
      if (start_idx < 0)
        break; // end of path reached
        
      auto start = m_waypoints[start_idx];

      // Compute the locally closest point and its distance.
      // - either the current waypoint or the closest point on the segment which starts at this waypoint.
      PointRef local_closest_pt(start_idx);
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
        local_closest_pt.t = t;
      }

      // Evaluate whether this is the new closest point.
      if (d <= min_dist) {
        min_dist = d;
        closest_point = local_closest_pt;
      }

      // Perform the threshold logic.
      // If we already entered the threshold region,
      if (new_window_start.has_value()) {
        // If we're now leaving it, then stop traversing the path
        if (d > threshold)
          break;
      } else {
        // Otherwise, if we're now entering it
        if (d <= threshold)
          new_window_start = WaypointRef(start_idx); // store the start waypoint.
      }
    }

    // If we never entered the window (new_window_start is nullopt), the vehicle is outside the boundaries. Keep the old window start.
    return ProjectionResult(closest_point, new_window_start.value_or(last_window_start));
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

  WaypointT get_position(const WaypointRef& at) const {
    assert(at.idx >= 0 && at.idx < n_waypoints());
    return m_waypoints[at.idx];
  }

  /**
   * Get the path position at the specified reference.
   * @param at The location.
   */
  WaypointT get_position(const PointRef& at) const {
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