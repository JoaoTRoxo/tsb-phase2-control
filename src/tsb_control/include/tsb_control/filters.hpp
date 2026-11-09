// Copyright 2026 Joao Roxo
/// @file filters.hpp
/// @brief Signal conditioning used by sensor_processing_node. No ROS dependency.
#ifndef TSB_CONTROL__FILTERS_HPP_
#define TSB_CONTROL__FILTERS_HPP_

#include <optional>

#include "tsb_control/angles.hpp"

namespace tsb_control
{

/// First-order low-pass filter, y' = (x - y) / tau, discretised with backward Euler.
class LowPassFilter
{
public:
  explicit LowPassFilter(double tau = 0.0)
  : tau_(tau) {}

  /// Feed one sample. The first sample initialises the output, so there is no
  /// artificial ramp from zero when the node starts while the vehicle is moving.
  double update(double input, double dt)
  {
    if (!value_ || tau_ <= 0.0) {
      value_ = input;
    } else if (dt > 0.0) {
      *value_ += dt / (tau_ + dt) * (input - *value_);
    }
    return *value_;
  }

  void set_time_constant(double tau) {tau_ = tau;}
  double value() const {return value_.value_or(0.0);}

private:
  double tau_;
  std::optional<double> value_{};
};

/// Complementary filter that fuses a gyroscope with an absolute heading.
///
/// The gyro is fast and smooth but its integral drifts; the odometry yaw does not
/// drift in the same way but is slower and coarser. Integrating the gyro and
/// pulling gently towards the absolute heading keeps the good half of each: a
/// high-pass on the gyro and a low-pass on the absolute heading, crossing over at
/// 1 / tau rad/s. Every correction is wrapped so the +-pi seam has no effect.
class HeadingFilter
{
public:
  explicit HeadingFilter(double tau = 1.0)
  : tau_(tau) {}

  /// Advance the estimate with a gyro sample. No-op before the first correction.
  void predict(double yaw_rate, double dt)
  {
    if (heading_ && dt > 0.0) {
      heading_ = wrap_pi(*heading_ + yaw_rate * dt);
    }
  }

  /// Pull the estimate towards an absolute heading measured @p dt after the last one.
  void correct(double measured_heading, double dt)
  {
    if (!heading_ || tau_ <= 0.0) {
      heading_ = wrap_pi(measured_heading);
      return;
    }
    if (dt > 0.0) {
      const double alpha = dt / (tau_ + dt);
      heading_ = wrap_pi(*heading_ + alpha * wrap_pi(measured_heading - *heading_));
    }
  }

  void set_time_constant(double tau) {tau_ = tau;}
  bool initialised() const {return heading_.has_value();}
  double heading() const {return heading_.value_or(0.0);}

private:
  double tau_;
  std::optional<double> heading_{};
};

}  // namespace tsb_control

#endif  // TSB_CONTROL__FILTERS_HPP_
