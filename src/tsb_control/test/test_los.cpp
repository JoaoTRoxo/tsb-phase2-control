// Copyright 2026 Joao Roxo
// Unit tests for line-of-sight guidance and waypoint switching.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tsb_control/angles.hpp"
#include "tsb_control/los.hpp"

using tsb_control::kPi;
using tsb_control::LosGuidance;
using tsb_control::path_errors;
using tsb_control::PathTracker;
using tsb_control::Waypoint;
using tsb_control::wrap_pi;

TEST(PathErrors, LeftOfThePathIsPositive)
{
  const auto p = path_errors({0.0, 0.0}, {2.0, 0.0}, 0.5, 0.25);
  EXPECT_NEAR(p.along, 0.5, 1e-12);
  EXPECT_NEAR(p.cross, 0.25, 1e-12);
  EXPECT_NEAR(p.alpha, 0.0, 1e-12);
  EXPECT_NEAR(p.length, 2.0, 1e-12);
}

TEST(PathErrors, WorksOnADiagonalSegment)
{
  const auto p = path_errors({0.0, 0.0}, {1.0, 1.0}, 1.0, 0.0);
  EXPECT_NEAR(p.alpha, kPi / 4.0, 1e-12);
  EXPECT_NEAR(p.along, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(p.cross, -std::sqrt(0.5), 1e-12);
}

TEST(Los, OnThePathPointsAlongThePath)
{
  LosGuidance los(0.5, 0.0);
  const auto p = path_errors({0.0, 0.0}, {0.0, 3.0}, 0.0, 1.0);
  EXPECT_NEAR(los.update(p, 0.05), kPi / 2.0, 1e-12);
}

TEST(Los, TurnsBackTowardsThePath)
{
  LosGuidance los(0.5, 0.0);
  const auto left = path_errors({0.0, 0.0}, {3.0, 0.0}, 1.0, 0.5);
  EXPECT_NEAR(los.update(left, 0.05), -kPi / 4.0, 1e-12);
  const auto right = path_errors({0.0, 0.0}, {3.0, 0.0}, 1.0, -0.5);
  EXPECT_NEAR(los.update(right, 0.05), kPi / 4.0, 1e-12);
}

TEST(Los, IntegralTermBuildsUpOnASteadyError)
{
  LosGuidance classic(0.5, 0.0);
  LosGuidance integral(0.5, 0.2);
  const auto p = path_errors({0.0, 0.0}, {3.0, 0.0}, 1.0, 0.1);
  const double base = classic.update(p, 0.05);
  double last = 0.0;
  for (int k = 0; k < 100; ++k) {
    last = integral.update(p, 0.05);
  }
  EXPECT_LT(last, base);
}

TEST(PathTracker, RejectsBadInput)
{
  EXPECT_THROW(PathTracker({{0.0, 0.0}}, 0.4, 0.0, 0.1), std::invalid_argument);
  EXPECT_THROW(PathTracker({{0.0, 0.0}, {1.0, 0.0}}, 0.0, 0.0, 0.1), std::invalid_argument);
}

TEST(PathTracker, SwitchesInsideTheAcceptanceCircle)
{
  PathTracker tracker({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, 0.4, 0.0, 0.1);
  EXPECT_EQ(tracker.update(0.5, 0.0, 0.05).segment, 0u);
  EXPECT_EQ(tracker.update(0.95, 0.0, 0.05).segment, 1u);
}

TEST(PathTracker, SwitchesWhenAWaypointIsOvershotOutsideTheCircle)
{
  PathTracker tracker({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, 0.4, 0.0, 0.1);
  // Beyond the end of the first segment, but 0.5 m to its side.
  EXPECT_EQ(tracker.update(1.05, 0.5, 0.05).segment, 1u);
}

TEST(PathTracker, FinishesAtTheLastWaypointAndHoldsHeading)
{
  PathTracker tracker({{0.0, 0.0}, {1.0, 0.0}}, 0.4, 0.0, 0.1);
  const auto running = tracker.update(0.5, 0.0, 0.05);
  EXPECT_FALSE(running.finished);
  const auto done = tracker.update(0.95, 0.0, 0.05);
  EXPECT_TRUE(done.finished);
  EXPECT_DOUBLE_EQ(done.heading, running.heading);
}

TEST(PathTracker, DistanceToGoalIncludesLaterSegments)
{
  PathTracker tracker({{0.0, 0.0}, {1.0, 0.0}, {1.0, 2.0}}, 0.4, 0.0, 0.1);
  EXPECT_NEAR(tracker.update(0.25, 0.0, 0.05).distance_to_goal, 2.75, 1e-12);
}
