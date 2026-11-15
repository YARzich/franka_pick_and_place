// Responsibility: Verifies name-based extraction and validation of Panda finger state and effort.

#include <gtest/gtest.h>

#include <limits>

#include "franka_motion/panda_joint_state_utils.hpp"

namespace
{
sensor_msgs::msg::JointState fingerState()
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"panda_joint1", "panda_finger_joint2", "panda_finger_joint1"};
  msg.position = {0.5, 0.03, 0.02};
  msg.effort = {1.0, -3.0, 2.0};
  return msg;
}
}  // namespace

TEST(PandaJointStateUtils, ExtractsPositionsAndAbsoluteEffort)
{
  const auto sample = franka_motion::parsePandaFingerJointState(fingerState());

  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->left_position, 0.02);
  EXPECT_DOUBLE_EQ(sample->right_position, 0.03);
  ASSERT_TRUE(sample->absolute_effort_sum.has_value());
  EXPECT_DOUBLE_EQ(sample->absolute_effort_sum.value(), 5.0);
}

TEST(PandaJointStateUtils, RequiresBothFiniteFingerPositions)
{
  auto msg = fingerState();
  msg.name.pop_back();
  EXPECT_FALSE(franka_motion::parsePandaFingerJointState(msg).has_value());

  msg = fingerState();
  msg.position[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(franka_motion::parsePandaFingerJointState(msg).has_value());

  msg = fingerState();
  msg.position.resize(2);
  EXPECT_FALSE(franka_motion::parsePandaFingerJointState(msg).has_value());
}

TEST(PandaJointStateUtils, OmitsIncompleteOrNonFiniteEffort)
{
  auto msg = fingerState();
  msg.effort.resize(2);
  auto sample = franka_motion::parsePandaFingerJointState(msg);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->absolute_effort_sum.has_value());

  msg = fingerState();
  msg.effort[1] = std::numeric_limits<double>::infinity();
  sample = franka_motion::parsePandaFingerJointState(msg);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->absolute_effort_sum.has_value());
}
