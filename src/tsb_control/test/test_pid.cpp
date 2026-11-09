// Copyright 2026 Joao Roxo
// Unit tests for the PID core and gain schedule.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tsb_control/gain_schedule.hpp"
#include "tsb_control/pid.hpp"

using tsb_control::AntiWindup;
using tsb_control::GainSchedule;
using tsb_control::Pid;
using tsb_control::PidConfig;
using tsb_control::PidGains;

namespace
{

PidConfig make_config(PidGains gains, double limit, AntiWindup anti_windup)
{
  PidConfig config;
  config.gains = gains;
  config.output_min = -limit;
  config.output_max = limit;
  config.anti_windup = anti_windup;
  return config;
}

}  // namespace

TEST(Pid, ProportionalTermIsGainTimesError)
{
  Pid pid(make_config({2.0, 0.0, 0.0}, 100.0, AntiWindup::kNone));
  EXPECT_DOUBLE_EQ(pid.update(1.5, 0.02).output, 3.0);
}

TEST(Pid, IntegralFollowsEulerSum)
{
  Pid pid(make_config({0.0, 2.0, 0.0}, 100.0, AntiWindup::kNone));
  for (int k = 0; k < 10; ++k) {
    pid.update(1.0, 0.1);
  }
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);
}

TEST(Pid, DerivativeIsBackwardDifferenceAndSkipsFirstSample)
{
  Pid pid(make_config({0.0, 0.0, 1.0}, 100.0, AntiWindup::kNone));
  EXPECT_DOUBLE_EQ(pid.update(5.0, 0.1).d, 0.0);
  EXPECT_NEAR(pid.update(6.0, 0.1).d, 10.0, 1e-9);
}

TEST(Pid, MeasuredRateReplacesFiniteDifference)
{
  Pid pid(make_config({0.0, 0.0, 2.0}, 100.0, AntiWindup::kNone));
  pid.update(0.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.update(50.0, 0.1, 0.0, -0.5).d, -1.0);
}

TEST(Pid, FeedforwardAddsToOutputAndRespectsLimit)
{
  Pid pid(make_config({1.0, 0.0, 0.0}, 1.0, AntiWindup::kNone));
  EXPECT_DOUBLE_EQ(pid.update(0.25, 0.1, 0.5).output, 0.75);
  EXPECT_DOUBLE_EQ(pid.update(0.25, 0.1, 5.0).output, 1.0);
}

TEST(Pid, UnreachableFeedforwardDoesNotDragTheIntegratorNegative)
{
  // Kt = Ki / Kp: the integrator should stay empty while the reference is out of reach.
  Pid pid(make_config({1.0, 0.5, 0.0}, 0.22, AntiWindup::kBackCalculation));
  for (int k = 0; k < 600; ++k) {
    pid.update(0.13, 0.02, 0.35);
  }
  EXPECT_NEAR(pid.integral(), 0.0, 1e-9);
}

TEST(Pid, OutputIsClampedAndFlagged)
{
  Pid pid(make_config({10.0, 0.0, 0.0}, 1.0, AntiWindup::kNone));
  const auto result = pid.update(1.0, 0.1);
  EXPECT_DOUBLE_EQ(result.output, 1.0);
  EXPECT_TRUE(result.saturated);
}

TEST(Pid, NonPositiveDtDoesNothing)
{
  Pid pid(make_config({1.0, 1.0, 0.0}, 10.0, AntiWindup::kNone));
  pid.update(1.0, 0.1);
  const double integral = pid.integral();
  pid.update(1.0, 0.0);
  pid.update(1.0, -1.0);
  EXPECT_DOUBLE_EQ(pid.integral(), integral);
}

TEST(Pid, ResetClearsHistory)
{
  Pid pid(make_config({0.0, 1.0, 1.0}, 10.0, AntiWindup::kNone));
  pid.update(1.0, 0.1);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
  EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1).d, 0.0);
}

TEST(Pid, ChangingKiDoesNotJumpTheOutput)
{
  // The integrator stores the term rather than the error sum, so that a gain
  // schedule can change Ki while the loop runs.
  Pid pid(make_config({0.0, 1.0, 0.0}, 100.0, AntiWindup::kNone));
  for (int k = 0; k < 10; ++k) {
    pid.update(1.0, 0.1);
  }
  const double before = pid.update(0.0, 0.1).output;
  pid.set_gains({0.0, 5.0, 0.0});
  EXPECT_DOUBLE_EQ(pid.update(0.0, 0.1).output, before);
}

TEST(AntiWindup, NoneWindsUpWhileSaturated)
{
  Pid pid(make_config({0.0, 1.0, 0.0}, 0.5, AntiWindup::kNone));
  for (int k = 0; k < 100; ++k) {
    pid.update(1.0, 0.1);
  }
  EXPECT_NEAR(pid.integral(), 10.0, 1e-9);
}

TEST(AntiWindup, ClampStopsAtTheLimitAndStillUnwinds)
{
  Pid pid(make_config({0.0, 1.0, 0.0}, 0.5, AntiWindup::kClamp));
  for (int k = 0; k < 100; ++k) {
    pid.update(1.0, 0.1);
  }
  EXPECT_LE(pid.integral(), 0.6);
  const double wound = pid.integral();
  pid.update(-1.0, 0.1);
  EXPECT_LT(pid.integral(), wound);
}

TEST(AntiWindup, BackCalculationSettlesNearTheLimit)
{
  auto config = make_config({0.0, 1.0, 0.0}, 0.5, AntiWindup::kBackCalculation);
  config.tracking_gain = 5.0;
  Pid pid(config);
  for (int k = 0; k < 1000; ++k) {
    pid.update(1.0, 0.01);
  }
  // Equilibrium: Ki e = Kt (I - limit), so I = limit + Ki e / Kt = 0.7.
  EXPECT_NEAR(pid.integral(), 0.7, 1e-3);
}

TEST(AntiWindup, IntegratorHoldsWhileTheAllocationCutsThisLoop)
{
  // The loop's own limit is never reached, but the allocation only delivers half.
  Pid pid(make_config({1.0, 1.0, 0.0}, 100.0, AntiWindup::kBackCalculation));
  for (int k = 0; k < 500; ++k) {
    pid.update(1.0, 0.01, 0.0, std::nullopt, 0.5);
  }
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(AntiWindup, IntegratorCanStillUnwindDuringAnAllocationCut)
{
  Pid pid(make_config({1.0, 1.0, 0.0}, 100.0, AntiWindup::kBackCalculation));
  pid.update(1.0, 1.0);
  const double wound = pid.integral();
  // The output is still positive, but the error now asks for less.
  pid.update(-0.1, 0.1, 5.0, std::nullopt, 0.5);
  EXPECT_LT(pid.integral(), wound);
}

TEST(Pid, IntegralZoneOnlyIntegratesNearTheSetpoint)
{
  auto config = make_config({0.0, 1.0, 0.0}, 100.0, AntiWindup::kNone);
  config.integral_zone = 0.5;
  Pid pid(config);
  pid.update(1.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
  pid.update(0.25, 0.1);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.025);
}

TEST(AntiWindup, ZeroKiKeepsTheIntegratorEmpty)
{
  Pid pid(make_config({10.0, 0.0, 0.0}, 0.5, AntiWindup::kBackCalculation));
  pid.update(1.0, 0.1);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(GainSchedule, InterpolatesAndHoldsEnds)
{
  const GainSchedule schedule({0.0, 1.0}, {PidGains{2.0, 0.0, 0.0}, PidGains{4.0, 1.0, 2.0}});
  EXPECT_DOUBLE_EQ(schedule.at(-1.0).kp, 2.0);
  EXPECT_DOUBLE_EQ(schedule.at(0.5).kp, 3.0);
  EXPECT_DOUBLE_EQ(schedule.at(0.5).ki, 0.5);
  EXPECT_DOUBLE_EQ(schedule.at(0.5).kd, 1.0);
  EXPECT_DOUBLE_EQ(schedule.at(9.0).kp, 4.0);
}

TEST(GainSchedule, UsesTheCorrectIntervalInALongerTable)
{
  const GainSchedule schedule(
    {0.0, 1.0, 3.0}, {PidGains{0.0, 0.0, 0.0}, PidGains{1.0, 0.0, 0.0}, PidGains{5.0, 0.0, 0.0}});
  EXPECT_DOUBLE_EQ(schedule.at(2.0).kp, 3.0);
}

TEST(GainSchedule, RejectsBadTables)
{
  EXPECT_THROW(GainSchedule({}, {}), std::invalid_argument);
  EXPECT_THROW(
    GainSchedule({0.0, 1.0}, std::vector<PidGains>(1)), std::invalid_argument);
  EXPECT_THROW(
    GainSchedule({1.0, 1.0}, std::vector<PidGains>(2)), std::invalid_argument);
}
