// Copyright 2026 Joao Roxo
// Unit tests for the differential-drive allocation.

#include <gtest/gtest.h>

#include <cmath>

#include "tsb_control/allocation.hpp"

using tsb_control::allocate;
using tsb_control::DiffDriveGeometry;

namespace
{
const DiffDriveGeometry kGeometry{};   // TurtleBot3 Burger defaults
const double kRimMax = kGeometry.wheel_radius * kGeometry.max_wheel_speed;
const double kHalfTrack = kGeometry.wheel_separation / 2.0;
}  // namespace

TEST(Allocation, StraightLineDrivesBothWheelsEqually)
{
  const auto a = allocate(0.1, 0.0, kGeometry);
  EXPECT_DOUBLE_EQ(a.left, a.right);
  EXPECT_NEAR(a.left, 0.1 / kGeometry.wheel_radius, 1e-12);
  EXPECT_FALSE(a.saturated);
}

TEST(Allocation, PositiveYawSpeedsUpTheRightWheel)
{
  const auto a = allocate(0.0, 1.0, kGeometry);
  EXPECT_GT(a.right, 0.0);
  EXPECT_DOUBLE_EQ(a.left, -a.right);
}

TEST(Allocation, WheelSpeedsReconstructTheCommand)
{
  const auto a = allocate(0.1, 0.5, kGeometry);
  const double surge = kGeometry.wheel_radius * (a.left + a.right) / 2.0;
  const double yaw = kGeometry.wheel_radius * (a.right - a.left) / kGeometry.wheel_separation;
  EXPECT_NEAR(surge, 0.1, 1e-12);
  EXPECT_NEAR(yaw, 0.5, 1e-12);
}

TEST(Allocation, YawHasPriorityWhenBothDoNotFit)
{
  const auto a = allocate(0.22, 1.0, kGeometry);
  EXPECT_DOUBLE_EQ(a.yaw, 1.0);
  EXPECT_NEAR(a.surge, kRimMax - 1.0 * kHalfTrack, 1e-12);
  EXPECT_LT(a.surge_ratio, 1.0);
  EXPECT_DOUBLE_EQ(a.yaw_ratio, 1.0);
  EXPECT_TRUE(a.saturated);
}

TEST(Allocation, WheelsNeverExceedTheLimit)
{
  for (double surge = -1.0; surge <= 1.0; surge += 0.1) {
    for (double yaw = -10.0; yaw <= 10.0; yaw += 0.5) {
      const auto a = allocate(surge, yaw, kGeometry);
      EXPECT_LE(std::abs(a.left), kGeometry.max_wheel_speed + 1e-9);
      EXPECT_LE(std::abs(a.right), kGeometry.max_wheel_speed + 1e-9);
    }
  }
}

TEST(Allocation, ZeroCommandReportsNoSaturation)
{
  const auto a = allocate(0.0, 0.0, kGeometry);
  EXPECT_DOUBLE_EQ(a.surge_ratio, 1.0);
  EXPECT_DOUBLE_EQ(a.yaw_ratio, 1.0);
  EXPECT_FALSE(a.saturated);
}
