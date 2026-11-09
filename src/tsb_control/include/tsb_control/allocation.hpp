// Copyright 2026 Joao Roxo
/// @file allocation.hpp
/// @brief Differential-drive allocation with yaw priority. No ROS dependency.
#ifndef TSB_CONTROL__ALLOCATION_HPP_
#define TSB_CONTROL__ALLOCATION_HPP_

#include <algorithm>
#include <cmath>

namespace tsb_control
{

/// Differential-drive geometry and actuator limit.
struct DiffDriveGeometry
{
  double wheel_radius{0.033};      ///< [m]
  double wheel_separation{0.160};  ///< [m]
  double max_wheel_speed{6.67};    ///< [rad/s]
};

/// Result of one allocation.
struct Allocation
{
  double left{0.0};         ///< [rad/s] left wheel speed
  double right{0.0};        ///< [rad/s] right wheel speed
  double surge{0.0};        ///< [m/s] surge actually delivered
  double yaw{0.0};          ///< [rad/s] yaw rate actually delivered
  double surge_ratio{1.0};  ///< delivered / requested surge
  double yaw_ratio{1.0};    ///< delivered / requested yaw rate
  bool saturated{false};
};

/// Map a [surge, yaw rate] command to wheel speeds, respecting the wheel limit.
///
/// Both channels share the same two wheels: v_{L,R} = v -+ w b / 2. When they do
/// not fit, yaw is served first and surge gets what is left. Holding the heading
/// matters more than holding the speed, since an uncontrolled heading is what
/// takes the vehicle off the path.
inline Allocation allocate(double surge_cmd, double yaw_cmd, const DiffDriveGeometry & geometry)
{
  const double half_track = geometry.wheel_separation / 2.0;
  const double rim_speed_max = geometry.wheel_radius * geometry.max_wheel_speed;

  Allocation a;
  const double yaw_max = rim_speed_max / half_track;
  a.yaw = std::clamp(yaw_cmd, -yaw_max, yaw_max);

  const double surge_room = rim_speed_max - std::abs(a.yaw) * half_track;
  a.surge = std::clamp(surge_cmd, -surge_room, surge_room);

  a.left = (a.surge - a.yaw * half_track) / geometry.wheel_radius;
  a.right = (a.surge + a.yaw * half_track) / geometry.wheel_radius;

  a.surge_ratio = surge_cmd != 0.0 ? a.surge / surge_cmd : 1.0;
  a.yaw_ratio = yaw_cmd != 0.0 ? a.yaw / yaw_cmd : 1.0;
  a.saturated = a.surge_ratio < 1.0 || a.yaw_ratio < 1.0;
  return a;
}

}  // namespace tsb_control

#endif  // TSB_CONTROL__ALLOCATION_HPP_
