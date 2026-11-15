// Responsibility: Validate V2 transfer intent before observation or side effects.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_GOAL_VALIDATION_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_GOAL_VALIDATION_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "manipulation_msgs/action/transfer_object_sequence_v2.hpp"
#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"

namespace manipulation::workflow
{

struct TransferObjectSequenceV2GoalValidationPolicy
{
  std::size_t max_step_count{32U};
  std::string planning_frame;
  std::vector<std::uint8_t> supported_recovery_policies{
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_NONE,
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_SAFE_STOP};
};

struct TransferObjectSequenceV2GoalValidationResult
{
  bool valid{false};
  std::string reason;
};

[[nodiscard]] TransferObjectSequenceV2GoalValidationResult
validate_transfer_object_sequence_v2_step(
  const manipulation_msgs::msg::TransferObjectStepV2 & step,
  const TransferObjectSequenceV2GoalValidationPolicy & policy,
  bool has_previous_object_occurrence);

[[nodiscard]] TransferObjectSequenceV2GoalValidationResult
validate_transfer_object_sequence_v2_goal(
  const manipulation_msgs::action::TransferObjectSequenceV2::Goal & goal,
  const TransferObjectSequenceV2GoalValidationPolicy & policy);

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_GOAL_VALIDATION_HPP_
