#ifndef REFERENCE_PATH_HPP
#define REFERENCE_PATH_HPP

#include <functional>
#include <optional>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <span>
#include <fstream>
#include <ranges>

#include <control_node/path/geometry_helpers.hpp>

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

  struct PointData {
    using StorageT = Eigen::Vector2d;

    double dist_to_next;
    double curvature;

    friend ReferencePath;

    enum {
      DistToNext = 0,
      Curvature = 1
    };

  private:
    PointData(StorageT data) : dist_to_next(data(DistToNext)), curvature(data(Curvature)) {}
  };

private:
  std::span<Eigen::Vector2d> m_waypoints;
  std::span<PointData::StorageT> m_data;

  bool m_is_closed;
  bool m_is_data_valid;
  
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

  PointData get_data_impl(const PointRef& at) const {
    assert(at.prev_waypoint_idx >= 0 && at.prev_waypoint_idx < n_waypoints());

    std::optional<SegmentRef> seg = compute_segment(at.prev_waypoint_idx);
    if (!seg.has_value()) // There's no segment at that index (that waypoint doesn't have a successor)
      return PointData(m_data[at.prev_waypoint_idx]); // return the waypoint data itself

    // Interpolate the data for this segment
    return PointData(
      m_data[seg->start] + at.t * (m_data[seg->end] - m_data[seg->start])
    );
  }

public:
  struct PathProperties {
    bool is_closed;
    bool is_data_valid;
  };

  // Passing spans around because it must be readily apparent that ReferencePath is a stateless object.
  ReferencePath(std::span<Eigen::Vector2d> waypoints, std::span<PointData::StorageT> data, PathProperties prop) : m_waypoints(waypoints), m_data(data), m_is_closed(prop.is_closed), m_is_data_valid(prop.is_data_valid) {
    assert (waypoints.size() == data.size() && "The waypoints and data views must have the same size.");
    m_waypoints_size = (int)waypoints.size();
  }

  ReferencePath()
    : m_waypoints(std::span<Eigen::Vector2d>()),
      m_data(std::span<PointData::StorageT>()),
      m_is_closed(false),
      m_is_data_valid(false),
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

  /**
   * Get the path data at the specified reference.
   * @param at The location.
   */
  std::optional<PointData> get_data(const PointRef& at) const {
    if (n_waypoints() <= 0 || !is_data_valid())
      return {};

    return get_data_impl(at);
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

  void compute_data() {
    // Compute, for each waypoint, the length of the segment that connects it to the next waypoint
    for (int i = 0; i < n_waypoints(); ++i) {
      int next_i = compute_index(i, 1);
      m_data[i](PointData::DistToNext) = next_i >= 0? (m_waypoints[next_i] - m_waypoints[i]).norm() : NAN;
    }
  
    // Compute path curvature
    m_data[0](PointData::Curvature) = 0; // This remains 0 unless the path is closed.
    for (int prev_idx = 0; prev_idx < n_waypoints(); ++prev_idx) {
      int curr_idx = compute_index(prev_idx, 1);
      if (curr_idx == -1)
        break;
      
      int next_idx = compute_index(curr_idx, 1);
      if (next_idx == -1) {
        // Keep the last curvature.
        m_data[curr_idx](PointData::Curvature) = m_data[prev_idx](PointData::Curvature);
        break;
      }

      m_data[curr_idx](PointData::Curvature) = geometry_helpers::menger_curvature(m_waypoints[prev_idx], m_waypoints[curr_idx], m_waypoints[next_idx]);
    }
    
    // Smoothen path curvature
    int half_window_size = 10; //Meters in front and meters behind point at curr_idx (is an arbitrary value)

    std::vector<double> filtered_k;
    filtered_k.resize(m_data.size());

    //for each waypoint
    for (int curr_idx = 0; curr_idx < n_waypoints(); ++curr_idx) 
    { 
      // Compute the start and end of the window.
      PointRef window_start = trace_back_point(curr_idx, half_window_size); 
      PointRef window_end = advance_point(curr_idx, half_window_size);

      // Compute the first and last waypoints s.t. PointRef(first_waypoint_idx) >= window_start && PointRef(last_window_idx) <= window_end
      int last_waypoint_idx = window_end.prev_waypoint_idx;
      int first_waypoint_idx = compute_index(window_start.prev_waypoint_idx, 1);
      if (first_waypoint_idx < 0 || first_waypoint_idx >= last_waypoint_idx)
        first_waypoint_idx = last_waypoint_idx;

      // Compute the area under the curve and the cumulative distance, in order to later compute a weighted average.
      auto compute_area_fn = [](double fa, double fb, double ab) { return 0.5 * (fa + fb) * ab; };
      double area_under_curve = 0.0;
      double actual_window_width = 0.0;

      // The start and end of the window are (possibly) not waypoints.
      double d_start = (get_position(window_start) - m_waypoints[first_waypoint_idx]).norm();
      double d_end = (get_position(window_end) - m_waypoints[last_waypoint_idx]).norm();

      area_under_curve += compute_area_fn(get_data_impl(window_start).curvature, m_data[first_waypoint_idx](PointData::Curvature), d_start);
      area_under_curve += compute_area_fn(get_data_impl(window_end).curvature, m_data[last_waypoint_idx](PointData::Curvature), d_end);
      actual_window_width += d_start + d_end;

      for (int i = first_waypoint_idx; i < last_waypoint_idx; ++i) {
        int next_waypoint_idx = compute_index(i, 1);

        // We're inside the window, so it's impossible that we find the path end (otherwise the window would be malformed)
        assert(next_waypoint_idx > 0 && "Malformed window! At waypoint < window end but found the path end??");
        
        double d = m_data[i](PointData::DistToNext);
        actual_window_width += d;
        area_under_curve += compute_area_fn(m_data[i](PointData::Curvature), m_data[next_waypoint_idx](PointData::Curvature), d);
      }

      filtered_k[curr_idx] = area_under_curve / actual_window_width;
    }
    
    for (int i = 0; i < n_waypoints(); ++i)
      m_data[i](PointData::Curvature) = filtered_k[i];

    m_is_data_valid = true;
  }

  void dump(const std::string& path) {
    Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ");
    
    std::ofstream f(path);
    f << "x,y,seg_len,k\n";

    for (int i = 0; i < n_waypoints(); ++i)
      f << m_waypoints[i].format(CSVFormat) << ", " << m_data[i].format(CSVFormat) << "\n";
  }

  bool is_data_valid() const { return m_is_data_valid; }
  bool is_closed() const { return m_is_closed; } 
};

}; // namespace path
}; // namespace control_node

#endif 