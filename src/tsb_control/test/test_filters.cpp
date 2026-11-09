// Copyright 2026 Joao Roxo
// Unit tests for the sensor-processing filters.

#include <gtest/gtest.h>

#include <cmath>

#include "tsb_control/angles.hpp"
#include "tsb_control/filters.hpp"

using tsb_control::HeadingFilter;
using tsb_control::kPi;
using tsb_control::LowPassFilter;
using tsb_control::wrap_pi;
using tsb_control::yaw_from_quaternion;

TEST(Angles, WrapGivesShortestSignedTurn)
{
  EXPECT_NEAR(wrap_pi(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
  EXPECT_NEAR(wrap_pi(-3.0 * kPi / 2.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(wrap_pi(0.25), 0.25, 1e-12);
}

TEST(Angles, YawFromQuaternion)
{
  const double yaw = 1.2;
  EXPECT_NEAR(yaw_from_quaternion(0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)), yaw, 1e-12);
}

TEST(LowPass, FirstSampleInitialisesTheOutput)
{
  LowPassFilter filter(1.0);
  EXPECT_DOUBLE_EQ(filter.update(3.0, 0.0), 3.0);
}

TEST(LowPass, ReachesAboutSixtyThreePercentAfterOneTimeConstant)
{
  LowPassFilter filter(1.0);
  filter.update(0.0, 0.0);
  double output = 0.0;
  for (int k = 0; k < 1000; ++k) {
    output = filter.update(1.0, 0.001);
  }
  EXPECT_NEAR(output, 1.0 - std::exp(-1.0), 2e-3);
}

TEST(HeadingFilter, FirstCorrectionInitialises)
{
  HeadingFilter filter(1.0);
  EXPECT_FALSE(filter.initialised());
  filter.predict(1.0, 1.0);
  EXPECT_FALSE(filter.initialised());
  filter.correct(0.5, 0.0);
  EXPECT_DOUBLE_EQ(filter.heading(), 0.5);
}

TEST(HeadingFilter, GyroDrivesTheFastResponse)
{
  HeadingFilter filter(1.0);
  filter.correct(0.0, 0.0);
  for (int k = 0; k < 100; ++k) {
    filter.predict(1.0, 0.001);
  }
  EXPECT_NEAR(filter.heading(), 0.1, 1e-9);
}

TEST(HeadingFilter, CorrectionsRemoveGyroBias)
{
  HeadingFilter filter(0.5);
  filter.correct(0.0, 0.0);
  for (int k = 0; k < 20000; ++k) {
    filter.predict(0.01, 0.005);     // constant gyro bias
    if (k % 6 == 0) {
      filter.correct(0.0, 0.03);
    }
  }
  // The bias leaves a small bounded offset rather than an ever-growing one.
  EXPECT_LT(std::abs(filter.heading()), 0.01);
}

TEST(HeadingFilter, CorrectionTakesTheShortWayAcrossThePiSeam)
{
  HeadingFilter filter(1.0);
  filter.correct(kPi - 0.05, 0.0);
  filter.correct(-kPi + 0.05, 0.5);
  // Moved a little across the seam, rather than swinging through zero.
  EXPECT_GT(std::abs(filter.heading()), kPi - 0.05);
}
