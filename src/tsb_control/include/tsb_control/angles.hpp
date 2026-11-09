// Copyright 2026 Joao Roxo
/// @file angles.hpp
/// @brief Angle helpers shared by every node.
#ifndef TSB_CONTROL__ANGLES_HPP_
#define TSB_CONTROL__ANGLES_HPP_

#include <cmath>

namespace tsb_control
{

constexpr double kPi = 3.14159265358979323846;

/// Wrap an angle to [-pi, pi]. Applied to a difference of two angles, this gives
/// the shortest signed turn between them, which is what stops a heading loop from
/// spinning the long way round when the setpoint crosses +-180 degrees.
inline double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Yaw of a unit quaternion (rotation about z, counter-clockwise positive).
inline double yaw_from_quaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace tsb_control

#endif  // TSB_CONTROL__ANGLES_HPP_
