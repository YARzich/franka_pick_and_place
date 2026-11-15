// Responsibility: Verifies gripper width conversion, command validation, and stall detection.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>

#include "franka_motion/gripper_utils.hpp"

TEST(GripperUtils, ClampsWidthToLimits)
{
  EXPECT_DOUBLE_EQ(franka_motion::clampGripperWidth(-0.1), 0.0);
  EXPECT_DOUBLE_EQ(franka_motion::clampGripperWidth(0.0), 0.0);
  EXPECT_DOUBLE_EQ(franka_motion::clampGripperWidth(0.08), 0.08);
  EXPECT_DOUBLE_EQ(franka_motion::clampGripperWidth(0.2), 0.08);
}

TEST(GripperUtils, ConvertsWidthToJointValue)
{
  EXPECT_DOUBLE_EQ(franka_motion::widthToJoint(0.0), 0.0);
  EXPECT_DOUBLE_EQ(franka_motion::widthToJoint(0.08), 0.04);
  EXPECT_DOUBLE_EQ(franka_motion::widthToJoint(0.1), 0.04);
}

TEST(GripperUtils, ValidatesCommandsAndWidthState)
{
  EXPECT_TRUE(franka_motion::isValidGripperCommand(0.04, 20.0));
  EXPECT_FALSE(franka_motion::isValidGripperCommand(
      std::numeric_limits<double>::quiet_NaN(), 20.0));
  EXPECT_FALSE(franka_motion::isValidGripperCommand(0.04, -1.0));
  EXPECT_TRUE(franka_motion::positionOnlyGripperAcceptsMaxEffort(0.0));
  EXPECT_FALSE(franka_motion::positionOnlyGripperAcceptsMaxEffort(0.001));
  EXPECT_FALSE(franka_motion::positionOnlyGripperAcceptsMaxEffort(
      std::numeric_limits<double>::infinity()));

  EXPECT_TRUE(franka_motion::gripperWidthReached(0.041, 0.04, 0.002));
  EXPECT_FALSE(franka_motion::gripperWidthReached(0.05, 0.04, 0.002));
  EXPECT_TRUE(franka_motion::isClosingGripperCommand(0.03, 0.0, 0.002));
  EXPECT_FALSE(franka_motion::isClosingGripperCommand(0.03, 0.08, 0.002));

  EXPECT_TRUE(franka_motion::gripperStallHasClosingProgress(
      0.08, 0.04, 0.0, 0.001, 0.002));
  EXPECT_FALSE(franka_motion::gripperStallHasClosingProgress(
      0.08, 0.08, 0.0, 0.001, 0.002));
  EXPECT_FALSE(franka_motion::gripperStallHasClosingProgress(
      0.02, 0.02, 0.08, 0.001, 0.002));

  EXPECT_TRUE(franka_motion::gripperFingerMotionSupportsClosingContact(
      0.039, 0.039, 0.040, 0.011, 0.0005, 0.002));
  EXPECT_TRUE(franka_motion::gripperFingerMotionSupportsClosingContact(
      0.039, 0.039, 0.033, 0.017, 0.0005, 0.002));
  EXPECT_FALSE(franka_motion::gripperFingerMotionSupportsClosingContact(
      0.030, 0.030, 0.035, 0.005, 0.0005, 0.002));
  EXPECT_FALSE(franka_motion::gripperFingerMotionSupportsClosingContact(
      0.039, 0.039, 0.039, 0.039, 0.0005, 0.002));
}

TEST(GripperUtils, DetectsStallOnlyAfterAValidObservationWithoutProgress)
{
  using namespace std::chrono_literals;
  const auto start = std::chrono::steady_clock::time_point{};
  franka_motion::GripperProgressMonitor monitor(0.001);
  monitor.reset(start);

  EXPECT_FALSE(monitor.hasStalled(start + 1s, 500ms));
  EXPECT_FALSE(monitor.observe(std::numeric_limits<double>::quiet_NaN(), start));
  EXPECT_TRUE(monitor.observe(0.08, start));
  EXPECT_FALSE(monitor.hasStalled(start + 499ms, 500ms));
  EXPECT_TRUE(monitor.hasStalled(start + 500ms, 500ms));

  EXPECT_TRUE(monitor.observe(0.075, start + 500ms));
  EXPECT_FALSE(monitor.hasStalled(start + 750ms, 500ms));
  EXPECT_TRUE(monitor.hasStalled(start + 1s, 500ms));
}
