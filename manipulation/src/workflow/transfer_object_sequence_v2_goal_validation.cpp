// Responsibility: Enforce V2 object, target, ordering, and recovery invariants.

#include "manipulation/workflow/transfer_object_sequence_v2_goal_validation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::workflow
{
namespace
{

using Step = manipulation_msgs::msg::TransferObjectStepV2;
using Validation = TransferObjectSequenceV2GoalValidationResult;

Validation invalid(std::string reason)
{
  return {false, std::move(reason)};
}

bool valid_object_id(const std::string & value)
{
  return !value.empty() && value.find('|') == std::string::npos &&
         std::none_of(
    value.cbegin(), value.cend(), [](const unsigned char character) {
      return std::isspace(character) != 0 || std::iscntrl(character) != 0;
    });
}

bool known_recovery_policy(const std::uint8_t policy) noexcept
{
  return policy <= Step::RECOVERY_RETREAT_AND_HOME;
}

Validation validate_policy(const TransferObjectSequenceV2GoalValidationPolicy & policy)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(policy.planning_frame)) {
    return invalid("planning_frame must be a canonical relative frame ID");
  }
  if (policy.max_step_count == 0U ||
    policy.max_step_count > std::numeric_limits<std::uint32_t>::max())
  {
    return invalid("max_step_count must fit the positive uint32 range");
  }
  if (policy.supported_recovery_policies.empty() ||
    std::any_of(
      policy.supported_recovery_policies.cbegin(),
      policy.supported_recovery_policies.cend(),
      [](const std::uint8_t value) {return !known_recovery_policy(value);}))
  {
    return invalid("supported recovery policies must be non-empty and known");
  }
  return {true, {}};
}

bool default_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  return pose == geometry_msgs::msg::PoseStamped{};
}

bool default_vector(const geometry_msgs::msg::Vector3Stamped & vector)
{
  return vector == geometry_msgs::msg::Vector3Stamped{};
}

Validation absolute_target(const Step & step, const std::string & frame)
{
  if (!manipulation::validation::is_valid_pose_stamped(step.target_object_pose, frame)) {
    return invalid(
      "target_object_pose must use planning_frame with a normalized non-zero "
      "stamp, finite pose, and unit quaternion");
  }
  return default_vector(step.target_translation) ? Validation{true, {}} :
         invalid("target_translation must be empty for TARGET_ABSOLUTE_POSE");
}

Validation translated_target(const Step & step, const std::string & frame)
{
  if (!default_pose(step.target_object_pose)) {
    return invalid(
      "target_object_pose must be empty for TARGET_TRANSLATION_FROM_CURRENT_SOURCE");
  }
  const auto & translation = step.target_translation;
  if (translation.header.frame_id != frame ||
    !manipulation::validation::is_canonical_relative_frame_id(
      translation.header.frame_id))
  {
    return invalid("target_translation frame must exactly equal planning_frame");
  }
  if (!manipulation::validation::is_normalized_nonzero_stamp(
      translation.header.stamp))
  {
    return invalid("target_translation stamp must be normalized and non-zero");
  }
  if (!std::isfinite(translation.vector.x) ||
    !std::isfinite(translation.vector.y) ||
    !std::isfinite(translation.vector.z))
  {
    return invalid("target_translation vector must be finite");
  }
  return {true, {}};
}

Validation initial_target(const Step & step, const bool has_previous)
{
  if (!has_previous) {
    return invalid("TARGET_INITIAL_SOURCE requires an earlier step for this object_id");
  }
  if (!default_pose(step.target_object_pose) ||
    !default_vector(step.target_translation))
  {
    return invalid("TARGET_INITIAL_SOURCE requires empty target payloads");
  }
  return {true, {}};
}

}  // namespace

TransferObjectSequenceV2GoalValidationResult
validate_transfer_object_sequence_v2_step(
  const Step & step,
  const TransferObjectSequenceV2GoalValidationPolicy & policy,
  const bool has_previous_object_occurrence)
{
  const auto checked_policy = validate_policy(policy);
  if (!checked_policy.valid) {
    return checked_policy;
  }
  if (!valid_object_id(step.object_id)) {
    return invalid("object_id must be non-empty and contain no whitespace, control, or '|'");
  }
  if (!known_recovery_policy(step.recovery_policy)) {
    return invalid("recovery_policy is unknown");
  }
  if (std::find(
      policy.supported_recovery_policies.cbegin(),
      policy.supported_recovery_policies.cend(), step.recovery_policy) ==
    policy.supported_recovery_policies.cend())
  {
    return invalid("recovery_policy is not supported by the selected strategy");
  }

  switch (step.target_mode) {
    case Step::TARGET_ABSOLUTE_POSE:
      return absolute_target(step, policy.planning_frame);
    case Step::TARGET_TRANSLATION_FROM_CURRENT_SOURCE:
      return translated_target(step, policy.planning_frame);
    case Step::TARGET_INITIAL_SOURCE:
      return initial_target(step, has_previous_object_occurrence);
    default:
      return invalid("target_mode is unknown");
  }
}

TransferObjectSequenceV2GoalValidationResult
validate_transfer_object_sequence_v2_goal(
  const manipulation_msgs::action::TransferObjectSequenceV2::Goal & goal,
  const TransferObjectSequenceV2GoalValidationPolicy & policy)
{
  const auto checked_policy = validate_policy(policy);
  if (!checked_policy.valid) {
    return checked_policy;
  }
  if (goal.steps.empty()) {
    return invalid("steps must not be empty");
  }
  if (goal.steps.size() > policy.max_step_count) {
    return invalid("steps exceeds max_step_count " + std::to_string(policy.max_step_count));
  }

  std::unordered_set<std::string> observed;
  observed.reserve(goal.steps.size());
  for (std::size_t index = 0U; index < goal.steps.size(); ++index) {
    const auto & step = goal.steps[index];
    const auto checked = validate_transfer_object_sequence_v2_step(
      step, policy, observed.find(step.object_id) != observed.cend());
    if (!checked.valid) {
      return invalid("steps[" + std::to_string(index) + "]: " + checked.reason);
    }
    observed.insert(step.object_id);
  }
  return {true, {}};
}

}  // namespace manipulation::workflow
