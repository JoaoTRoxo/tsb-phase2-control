// Copyright 2026 Joao Roxo
/// @file gain_schedule.hpp
/// @brief Piecewise-linear gain schedule. No ROS dependency.
#ifndef TSB_CONTROL__GAIN_SCHEDULE_HPP_
#define TSB_CONTROL__GAIN_SCHEDULE_HPP_

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tsb_control/pid.hpp"

namespace tsb_control
{

/// Gains as a function of an operating point, here the surge speed. Between
/// breakpoints the gains are interpolated linearly, and outside the table they
/// are held at the nearest end, so a sensor spike cannot extrapolate them.
class GainSchedule
{
public:
  GainSchedule() = default;

  /// @param breakpoints strictly increasing operating points.
  /// @param gains       one entry per breakpoint.
  GainSchedule(std::vector<double> breakpoints, std::vector<PidGains> gains)
  : breakpoints_(std::move(breakpoints)), gains_(std::move(gains))
  {
    if (breakpoints_.empty() || breakpoints_.size() != gains_.size()) {
      throw std::invalid_argument("gain schedule needs one set of gains per breakpoint");
    }
    if (!std::is_sorted(
        breakpoints_.begin(), breakpoints_.end(), [](double a, double b) {return a <= b;}))
    {
      throw std::invalid_argument("gain schedule breakpoints must be strictly increasing");
    }
  }

  /// Interpolated gains at operating point @p x.
  PidGains at(double x) const
  {
    if (x <= breakpoints_.front()) {
      return gains_.front();
    }
    if (x >= breakpoints_.back()) {
      return gains_.back();
    }
    const auto upper = std::upper_bound(breakpoints_.begin(), breakpoints_.end(), x);
    const auto i = static_cast<std::size_t>(upper - breakpoints_.begin());
    const double t = (x - breakpoints_[i - 1]) / (breakpoints_[i] - breakpoints_[i - 1]);
    const auto lerp = [t](double a, double b) {return a + t * (b - a);};
    return PidGains{
      lerp(gains_[i - 1].kp, gains_[i].kp),
      lerp(gains_[i - 1].ki, gains_[i].ki),
      lerp(gains_[i - 1].kd, gains_[i].kd)};
  }

private:
  std::vector<double> breakpoints_{};
  std::vector<PidGains> gains_{};
};

}  // namespace tsb_control

#endif  // TSB_CONTROL__GAIN_SCHEDULE_HPP_
