// Responsibility: Verifies Panda gripper command normalization and controller command conversion.

#include <gtest/gtest.h>

#include "panda_gripper_commands.hpp"

TEST(PandaGripperCommands, EncodesBoundedOneAndTwoJointWidthCommands)
{
  const auto one_joint = franka_motion::makePandaGripperWidthCommand(
    0.08, 1, 0.001, 0.039);
  EXPECT_EQ(one_joint.data, (std::vector<double>{0.039}));

  const auto two_joints = franka_motion::makePandaGripperWidthCommand(
    0.04, 2, 0.0, 0.04);
  EXPECT_EQ(two_joints.data, (std::vector<double>{0.02, 0.02}));
}

TEST(PandaGripperCommands, PreservesAsymmetricHoldWithinConfiguredBounds)
{
  const auto hold = franka_motion::makePandaGripperHoldCommand(
    0.045, -0.001, 2, 0.001, 0.039);
  EXPECT_EQ(hold.data, (std::vector<double>{0.039, 0.001}));

  const auto single_joint_hold = franka_motion::makePandaGripperHoldCommand(
    0.014, 0.006, 1, 0.001, 0.039);
  EXPECT_EQ(single_joint_hold.data, (std::vector<double>{0.014}));
}
