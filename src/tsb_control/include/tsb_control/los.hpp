// Copyright 2026 Joao Roxo
/// @file los.hpp
/// @brief Line-of-sight path following over straight segments. No ROS dependency.
#ifndef TSB_CONTROL__LOS_HPP_
#define TSB_CONTROL__LOS_HPP_

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tsb_control/angles.hpp"

namespace tsb_control
{

/// A point on the path in the odometry frame [m].
struct Waypoint
{
  double x{0.0};
  double y{0.0};
};

/// Position of the vehicle relative to one straight segment.
struct PathErrors
{
  double along{0.0};   ///< [m] progress along the segment
  double cross{0.0};   ///< [m] positive when the vehicle is left of the path
  double alpha{0.0};   ///< [rad] direction of the segment
  double length{0.0};  ///< [m] length of the segment
};

/// Project the position (x, y) onto the segment a -> b.
inline PathErrors path_errors(const Waypoint & a, const Waypoint & b, double x, double y)
{
  PathErrors p;
  p.alpha = std::atan2(b.y - a.y, b.x - a.x);
  p.length = std::hypot(b.x - a.x, b.y - a.y);
  const double dx = x - a.x;
  const double dy = y - a.y;
  p.along = dx * std::cos(p.alpha) + dy * std::sin(p.alpha);
  p.cross = -dx * std::sin(p.alpha) + dy * std::cos(p.alpha);
  return p;
}

/// Lookahead-based line-of-sight guidance with an optional integral term.
///
///   psi_d = alpha + atan(-(e + sigma * y_int) / delta)
///   y_int' = delta * e / ((e + sigma * y_int)^2 + delta^2)
///
/// Aim at a point delta metres ahead on the path: a short delta converges fast but
/// can oscillate, and a long one is smooth but slow. sigma = 0 is classic LOS.
/// sigma > 0 (Borhaug, Pavlov and Pettersen, 2008) adds the integral state that
/// removes the steady cross-track offset a current or wind would cause on a boat.
class LosGuidance
{
public:
  LosGuidance(double lookahead, double sigma)
  : lookahead_(lookahead), sigma_(sigma) {}

  /// Desired heading for the given path errors; advances the integral state.
  double update(const PathErrors & p, double dt)
  {
    const double effective = p.cross + sigma_ * integral_;
    const double heading = wrap_pi(p.alpha + std::atan(-effective / lookahead_));
    if (sigma_ > 0.0) {
      integral_ += dt * lookahead_ * p.cross / (effective * effective + lookahead_ * lookahead_);
    }
    return heading;
  }

  void reset() {integral_ = 0.0;}

private:
  double lookahead_;
  double sigma_;
  double integral_{0.0};
};

/// Output of one PathTracker update.
struct PathCommand
{
  double heading{0.0};               ///< [rad] desired heading
  double cross_track{0.0};           ///< [m]
  double distance_to_waypoint{0.0};  ///< [m] to the end of the active segment
  double distance_to_goal{0.0};      ///< [m] along the remaining path
  std::size_t segment{0};
  bool finished{false};
};

/// Walks a polyline, switching segments and running LOS on the active one.
class PathTracker
{
public:
  /// @param waypoints         at least two points.
  /// @param lookahead         [m] LOS lookahead distance, positive.
  /// @param sigma             integral LOS gain, 0 for classic LOS.
  /// @param acceptance_radius [m] a waypoint counts as reached within this distance.
  PathTracker(
    std::vector<Waypoint> waypoints, double lookahead, double sigma, double acceptance_radius)
  : waypoints_(std::move(waypoints)), los_(lookahead, sigma), acceptance_(acceptance_radius)
  {
    if (waypoints_.size() < 2) {
      throw std::invalid_argument("a path needs at least two waypoints");
    }
    if (lookahead <= 0.0) {
      throw std::invalid_argument("lookahead must be positive");
    }
  }

  std::size_t segment_count() const {return waypoints_.size() - 1;}
  const std::vector<Waypoint> & waypoints() const {return waypoints_;}

  /// Update with the current position; @p dt is the time since the last update.
  PathCommand update(double x, double y, double dt)
  {
    advance_past_reached_waypoints(x, y);

    const auto errors = active_errors(x, y);
    PathCommand c;
    c.segment = segment_;
    c.cross_track = errors.cross;
    c.finished = finished_;
    c.distance_to_waypoint = distance_to_end_of_segment(x, y);
    c.distance_to_goal = c.distance_to_waypoint + length_after_segment();

    if (finished_) {
      c.heading = last_heading_;
    } else {
      c.heading = los_.update(errors, dt);
      last_heading_ = c.heading;
    }
    return c;
  }

private:
  PathErrors active_errors(double x, double y) const
  {
    return path_errors(waypoints_[segment_], waypoints_[segment_ + 1], x, y);
  }

  double distance_to_end_of_segment(double x, double y) const
  {
    const auto & end = waypoints_[segment_ + 1];
    return std::hypot(end.x - x, end.y - y);
  }

  double length_after_segment() const
  {
    double total = 0.0;
    for (std::size_t i = segment_ + 1; i + 1 < waypoints_.size(); ++i) {
      total += std::hypot(
        waypoints_[i + 1].x - waypoints_[i].x,
        waypoints_[i + 1].y - waypoints_[i].y);
    }
    return total;
  }

  // A waypoint is reached inside the acceptance circle, or once it has been passed
  // along the segment. The second test stops the vehicle turning back for a
  // waypoint it overshot just outside the circle.
  void advance_past_reached_waypoints(double x, double y)
  {
    while (!finished_) {
      const auto errors = active_errors(x, y);
      const bool reached = distance_to_end_of_segment(x, y) <= acceptance_ ||
        errors.along >= errors.length;
      if (!reached) {
        return;
      }
      if (segment_ + 1 >= segment_count()) {
        finished_ = true;
      } else {
        ++segment_;
        los_.reset();
      }
    }
  }

  std::vector<Waypoint> waypoints_;
  LosGuidance los_;
  double acceptance_;
  std::size_t segment_{0};
  bool finished_{false};
  double last_heading_{0.0};
};

}  // namespace tsb_control

#endif  // TSB_CONTROL__LOS_HPP_
