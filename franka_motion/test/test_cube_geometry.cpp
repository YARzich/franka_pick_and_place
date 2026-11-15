// Responsibility: Verifies strict finite and equal-edge cube dimension validation.

#include <array>
#include <limits>

#include "gtest/gtest.h"

#include "franka_motion/cube_geometry.hpp"

TEST(CubeGeometry, AcceptsFinitePositiveEqualEdges)
{
  EXPECT_TRUE(franka_motion::has_cubic_dimensions({0.05, 0.05, 0.05}));
  EXPECT_TRUE(franka_motion::has_cubic_dimensions({1.0, 1.0, 1.0}));
}

TEST(CubeGeometry, RejectsCuboidsAndInvalidDimensions)
{
  EXPECT_FALSE(franka_motion::has_cubic_dimensions({0.04, 0.05, 0.06}));
  EXPECT_FALSE(franka_motion::has_cubic_dimensions({0.05, 0.05, 0.0}));
  EXPECT_FALSE(franka_motion::has_cubic_dimensions({0.05, -0.05, 0.05}));
  EXPECT_FALSE(franka_motion::has_cubic_dimensions(
      {0.05, 0.05, std::numeric_limits<double>::infinity()}));
}

TEST(CubeGeometry, AppliesRelativeTolerance)
{
  const std::array<double, 3> dimensions{1.0, 1.0005, 1.0};
  EXPECT_TRUE(franka_motion::has_cubic_dimensions(dimensions, 0.001));
  EXPECT_FALSE(franka_motion::has_cubic_dimensions(dimensions, 0.0001));
  EXPECT_FALSE(franka_motion::has_cubic_dimensions(dimensions, -1.0));
}
