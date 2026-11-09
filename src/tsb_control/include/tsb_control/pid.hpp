// Copyright 2026 Joao Roxo
/// @file pid.hpp
/// @brief Discrete PID controller with feedforward and anti-windup. No ROS dependency.
#ifndef TSB_CONTROL__PID_HPP_
#define TSB_CONTROL__PID_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace tsb_control
{

/// Proportional, integral and derivative gains of one loop.
struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

/// What the integrator does while the actuator is saturated.
enum class AntiWindup
{
  kNone,             ///< Keep integrating. Only useful to show the problem.
  kClamp,            ///< Conditional integration: freeze while pushing further into the limit.
  kBackCalculation,  ///< Bleed the integrator with the gap between delivered and requested output.
};

/// Static configuration of one loop.
struct PidConfig
{
  PidGains gains{};
  double output_min{-std::numeric_limits<double>::infinity()};
  double output_max{std::numeric_limits<double>::infinity()};
  AntiWindup anti_windup{AntiWindup::kBackCalculation};
  double tracking_gain{0.0};          ///< K_t [1/s], back-calculation only; 0 means Ki / Kp.
  double derivative_filter_tau{0.0};  ///< [s] low-pass on the derivative; 0 disables it.
  double integral_zone{0.0};          ///< Integrate only while |error| is below this; 0 disables.
};

/// Every term of one update, for logging and tests.
struct PidResult
{
  double p{0.0};
  double i{0.0};
  double d{0.0};
  double feedforward{0.0};
  double unsaturated{0.0};
  double output{0.0};
  bool saturated{false};
};

/// Discrete PID: u(k) = Kp e(k) + I(k) + Kd de(k) + feedforward.
///
/// The integrator is Euler-discretised as in the project statement, but it stores
/// the integral *term* I = sum(Ki e dt) rather than the integral of the error.
/// Changing Ki then cannot make the output jump, which is what makes gain
/// scheduling and live retuning bumpless.
class Pid
{
public:
  Pid() = default;
  explicit Pid(const PidConfig & config)
  : config_(config) {}

  /// Replace the configuration, keeping the integrator and derivative state.
  void configure(const PidConfig & config) {config_ = config;}

  /// Replace only the gains (used by gain scheduling).
  void set_gains(const PidGains & gains) {config_.gains = gains;}

  const PidConfig & config() const {return config_;}

  /// Compute one control update.
  /// @param error            reference - measurement.
  /// @param dt               seconds since the previous update; must be positive.
  /// @param feedforward      added to the output and included in the saturation check.
  /// @param error_rate       time derivative of the error if it is measured directly
  ///                         (for example from a gyroscope). When absent, the backward
  ///                         difference (e(k) - e(k-1)) / dt is used.
  /// @param downstream_ratio fraction of the output the actuators delivered. Below 1
  ///                         means something after this loop saturated, and the
  ///                         integrator then holds rather than demanding even more.
  PidResult update(
    double error, double dt, double feedforward = 0.0,
    std::optional<double> error_rate = std::nullopt, double downstream_ratio = 1.0)
  {
    if (!(dt > 0.0)) {
      return last_;
    }

    // A finite difference has no meaning on the first sample, so it starts at zero
    // to avoid a derivative kick when the loop is switched on.
    double rate = 0.0;
    if (error_rate) {
      rate = *error_rate;
    } else if (previous_error_) {
      rate = (error - *previous_error_) / dt;
    }
    previous_error_ = error;

    if (config_.derivative_filter_tau > 0.0) {
      const double alpha = dt / (config_.derivative_filter_tau + dt);
      filtered_rate_ += alpha * (rate - filtered_rate_);
    } else {
      filtered_rate_ = rate;
    }

    PidResult r;
    r.p = config_.gains.kp * error;
    r.i = integral_;
    r.d = config_.gains.kd * filtered_rate_;
    // A feedforward beyond the limit can never be delivered. Left unclamped, it would
    // count as saturation the integrator has to undo, and drag the integrator negative.
    r.feedforward = std::clamp(feedforward, config_.output_min, config_.output_max);
    r.unsaturated = r.p + r.i + r.d + r.feedforward;
    r.output = std::clamp(r.unsaturated, config_.output_min, config_.output_max);

    // Two different kinds of saturation: this loop's own limit, and a cut made later
    // by the allocation stage because surge and yaw share the wheels.
    const double own_excess = r.output - r.unsaturated;
    const bool cut_downstream = downstream_ratio < 1.0 - 1e-9;
    r.saturated = std::abs(own_excess) > 1e-9 || cut_downstream;

    integrate(error, dt, own_excess, cut_downstream && r.output * error > 0.0);
    last_ = r;
    return r;
  }

  /// Forget the integrator and derivative history (used when the loop is disabled).
  void reset()
  {
    integral_ = 0.0;
    previous_error_.reset();
    filtered_rate_ = 0.0;
    last_ = PidResult{};
  }

  double integral() const {return integral_;}

private:
  void integrate(double error, double dt, double own_excess, bool held_downstream)
  {
    if (config_.gains.ki == 0.0) {
      integral_ = 0.0;
      return;
    }
    const bool inside_zone =
      config_.integral_zone <= 0.0 || std::abs(error) < config_.integral_zone;
    const double increment = inside_zone ? config_.gains.ki * error * dt : 0.0;

    switch (config_.anti_windup) {
      case AntiWindup::kNone:
        integral_ += increment;
        break;
      case AntiWindup::kClamp: {
          // own_excess and error have opposite signs when integrating would push
          // the output further past the limit.
          const bool pushing_further = own_excess * error < 0.0;
          if (!pushing_further && !held_downstream) {
            integral_ += increment;
          }
          break;
        }
      case AntiWindup::kBackCalculation:
        // A cut by the allocation stage is temporary and says nothing about what
        // the integrator should hold, so that case freezes instead of tracking.
        if (!held_downstream) {
          integral_ += increment + tracking_gain() * own_excess * dt;
        }
        break;
    }
  }

  /// With K_t = Ki / Kp, the proportional excess and the integration cancel exactly
  /// during saturation, so the integrator neither winds up nor swings the other way.
  double tracking_gain() const
  {
    if (config_.tracking_gain > 0.0) {
      return config_.tracking_gain;
    }
    return config_.gains.kp > 0.0 ? config_.gains.ki / config_.gains.kp : config_.gains.ki;
  }

  PidConfig config_{};
  double integral_{0.0};
  std::optional<double> previous_error_{};
  double filtered_rate_{0.0};
  PidResult last_{};
};

}  // namespace tsb_control

#endif  // TSB_CONTROL__PID_HPP_
