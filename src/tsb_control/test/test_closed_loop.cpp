// Copyright 2026 Joao Roxo
// Closed-loop tests: the controllers against simple models of the TurtleBot, so
// that behaviour (settling, wrapping, recovery) is checked without Gazebo.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tsb_control/allocation.hpp"
#include "tsb_control/angles.hpp"
#include "tsb_control/los.hpp"
#include "tsb_control/pid.hpp"

using tsb_control::allocate;
using tsb_control::AntiWindup;
using tsb_control::DiffDriveGeometry;
using tsb_control::kPi;
using tsb_control::PathTracker;
using tsb_control::Pid;
using tsb_control::PidConfig;
using tsb_control::wrap_pi;

namespace
{

constexpr double kDt = 0.02;

PidConfig surge_config(AntiWindup anti_windup)
{
  PidConfig config;
  config.gains = {1.0, 0.5, 0.0};
  config.output_min = -0.22;
  config.output_max = 0.22;
  config.anti_windup = anti_windup;
  return config;
}

PidConfig heading_config()
{
  PidConfig config;
  config.gains = {2.0, 0.05, 0.1};
  config.output_min = -2.75;
  config.output_max = 2.75;
  return config;
}

// First-order velocity lag, a simple stand-in for the wheel controllers.
double lag(double current, double command, double tau = 0.1)
{
  return current + kDt / tau * (command - current);
}

// Seconds for the surge to recover to 0.1 m/s after demanding more than the
// robot can do for 12 s.
double recovery_time(AntiWindup anti_windup)
{
  Pid pid(surge_config(anti_windup));
  double v = 0.0;
  const auto step = [&](double reference) {
      const auto r = pid.update(reference - v, kDt, reference);
      v = lag(v, r.output);
    };
  for (int k = 0; k < static_cast<int>(12.0 / kDt); ++k) {
    step(0.35);
  }
  int k = 0;
  while (std::abs(v - 0.1) > 0.01 && k < 5000) {
    step(0.1);
    ++k;
  }
  return k * kDt;
}

}  // namespace

TEST(ClosedLoop, SurgeReachesTheSetpoint)
{
  Pid pid(surge_config(AntiWindup::kBackCalculation));
  double v = 0.0;
  for (int k = 0; k < 500; ++k) {
    v = lag(v, pid.update(0.15 - v, kDt, 0.15).output);
  }
  EXPECT_NEAR(v, 0.15, 1e-3);
}

TEST(ClosedLoop, IntegralRejectsAPlantGainError)
{
  // A plant that only delivers 80 % of the command needs the integrator.
  Pid pid(surge_config(AntiWindup::kBackCalculation));
  double v = 0.0;
  for (int k = 0; k < 2000; ++k) {
    v = lag(v, 0.8 * pid.update(0.15 - v, kDt, 0.15).output);
  }
  EXPECT_NEAR(v, 0.15, 1e-3);
}

TEST(ClosedLoop, AntiWindupShortensRecovery)
{
  const double none = recovery_time(AntiWindup::kNone);
  const double clamp = recovery_time(AntiWindup::kClamp);
  const double back_calculation = recovery_time(AntiWindup::kBackCalculation);
  EXPECT_GT(none, 3.0 * clamp);
  EXPECT_GT(none, 3.0 * back_calculation);
}

TEST(ClosedLoop, HeadingTakesTheShortWayAcrossThePiSeam)
{
  Pid pid(heading_config());
  double psi = -150.0 * kPi / 180.0;
  double r = 0.0;
  const double target = 150.0 * kPi / 180.0;
  double largest_error = 0.0;
  for (int k = 0; k < 500; ++k) {
    const double error = wrap_pi(target - psi);
    largest_error = std::max(largest_error, std::abs(error));
    r = lag(r, pid.update(error, kDt, 0.0, -r).output);
    psi = wrap_pi(psi + r * kDt);
  }
  // Starts 60 degrees away and never goes further away than that.
  EXPECT_LT(largest_error, 61.0 * kPi / 180.0);
  // The small Ki leaves a slow tail (time constant Kp / Ki), so about a degree remains.
  EXPECT_NEAR(wrap_pi(target - psi), 0.0, 0.02);
}

TEST(ClosedLoop, FollowsASquareWithinAFewCentimetres)
{
  const std::vector<tsb_control::Waypoint> square{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}};
  PathTracker tracker(square, 0.4, 0.0, 0.08);
  Pid surge(surge_config(AntiWindup::kBackCalculation));
  Pid heading(heading_config());
  const DiffDriveGeometry geometry{};

  double x = 0.0, y = 0.0, psi = 0.0, v = 0.0, r = 0.0;
  bool finished = false;
  double straight_error = 0.0;
  for (int k = 0; k < 10000 && !finished; ++k) {
    const auto command = tracker.update(x, y, kDt);
    finished = command.finished;

    const double error = wrap_pi(command.heading - psi);
    const double alignment = std::max(0.0, std::cos(error));
    const double v_d = finished ? 0.0 : std::min(
      0.15 * alignment * alignment, std::max(0.03, 0.5 * command.distance_to_goal));

    const auto tau_u = surge.update(v_d - v, kDt, v_d);
    const auto tau_r = heading.update(error, kDt, 0.0, -r);
    const auto a = allocate(tau_u.output, tau_r.output, geometry);
    v = lag(v, a.surge);
    r = lag(r, a.yaw);
    psi = wrap_pi(psi + r * kDt);
    x += v * std::cos(psi) * kDt;
    y += v * std::sin(psi) * kDt;

    // Past the corner transient, the robot should be on the line.
    if (command.distance_to_waypoint > 0.6 && command.distance_to_waypoint < 1.0) {
      straight_error = std::max(straight_error, std::abs(command.cross_track));
    }
  }
  EXPECT_TRUE(finished);
  EXPECT_LT(std::hypot(x, y), 0.12);
  EXPECT_LT(straight_error, 0.05);
}
