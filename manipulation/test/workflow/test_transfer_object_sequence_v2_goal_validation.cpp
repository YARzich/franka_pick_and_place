// Responsibility: Verify the minimal V2 intent and server-owned data boundary.

#include "gtest/gtest.h"

#include "manipulation/workflow/transfer_object_sequence_v2_goal_validation.hpp"

namespace manipulation::workflow
{
namespace
{

using Action = manipulation_msgs::action::TransferObjectSequenceV2;
using Step = manipulation_msgs::msg::TransferObjectStepV2;

TransferObjectSequenceV2GoalValidationPolicy policy()
{
  TransferObjectSequenceV2GoalValidationPolicy result;
  result.planning_frame = "world";
  return result;
}

Step absolute_step()
{
  Step step;
  step.object_id = "cube_1";
  step.target_mode = Step::TARGET_ABSOLUTE_POSE;
  step.target_object_pose.header.frame_id = "world";
  step.target_object_pose.header.stamp.sec = 1;
  step.target_object_pose.pose.orientation.w = 1.0;
  step.recovery_policy = Step::RECOVERY_SAFE_STOP;
  return step;
}

TEST(TransferObjectSequenceV2GoalValidation, AcceptsMinimalIntentWithoutTimeout)
{
  Action::Goal goal;
  goal.steps.push_back(absolute_step());
  EXPECT_TRUE(validate_transfer_object_sequence_v2_goal(goal, policy()).valid);
}

TEST(TransferObjectSequenceV2GoalValidation, InitialSourceRequiresPriorSameObject)
{
  Action::Goal goal;
  auto return_step = absolute_step();
  return_step.target_mode = Step::TARGET_INITIAL_SOURCE;
  return_step.target_object_pose = {};
  goal.steps.push_back(return_step);
  EXPECT_FALSE(validate_transfer_object_sequence_v2_goal(goal, policy()).valid);

  goal.steps.insert(goal.steps.begin(), absolute_step());
  EXPECT_TRUE(validate_transfer_object_sequence_v2_goal(goal, policy()).valid);
}

TEST(TransferObjectSequenceV2GoalValidation, RejectsInactivePayloadAndUnsupportedRecovery)
{
  auto step = absolute_step();
  step.target_translation.header.frame_id = "world";
  step.target_translation.header.stamp.sec = 1;
  EXPECT_FALSE(validate_transfer_object_sequence_v2_step(step, policy(), false).valid);

  step = absolute_step();
  step.recovery_policy = Step::RECOVERY_RETREAT;
  EXPECT_FALSE(validate_transfer_object_sequence_v2_step(step, policy(), false).valid);
}

TEST(TransferObjectSequenceV2GoalValidation, RejectsMalformedObjectIdAndWrongFrame)
{
  auto step = absolute_step();
  step.object_id = "bad id";
  EXPECT_FALSE(validate_transfer_object_sequence_v2_step(step, policy(), false).valid);
  step = absolute_step();
  step.target_object_pose.header.frame_id = "map";
  EXPECT_FALSE(validate_transfer_object_sequence_v2_step(step, policy(), false).valid);
}

}  // namespace
}  // namespace manipulation::workflow
