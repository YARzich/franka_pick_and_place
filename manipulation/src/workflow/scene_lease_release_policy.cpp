// Responsibility: Implements conservative lease release based on task result and object
//   disposition.

#include "manipulation/workflow/scene_lease_release_policy.hpp"

#include "manipulation_msgs/msg/manipulation_task_result.hpp"
#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"

namespace manipulation::workflow
{

SceneLeaseReleaseDecision scene_lease_release_decision(
  const std::uint8_t final_disposition) noexcept
{
  using TaskResult = manipulation_msgs::msg::ManipulationTaskResult;
  switch (final_disposition) {
    case TaskResult::SAFE_IDLE:
      return scene_lease_release_decision(
        manipulation::task::ObjectDisposition::kSafeIdle);
    case TaskResult::OBJECT_AT_SOURCE:
      return scene_lease_release_decision(
        manipulation::task::ObjectDisposition::kObjectAtSource);
    case TaskResult::OBJECT_AT_TARGET:
      return scene_lease_release_decision(
        manipulation::task::ObjectDisposition::kObjectAtTarget);
    case TaskResult::DISPOSITION_UNKNOWN:
    case TaskResult::HOLDING_OBJECT:
    case TaskResult::SCENE_UNCERTAIN:
    default:
      return SceneLeaseReleaseDecision::kRetain;
  }
}

SceneLeaseReleaseDecision scene_lease_release_decision(
  const manipulation::task::ObjectDisposition final_disposition) noexcept
{
  using ObjectDisposition = manipulation::task::ObjectDisposition;
  switch (final_disposition) {
    case ObjectDisposition::kSafeIdle:
    case ObjectDisposition::kObjectAtSource:
    case ObjectDisposition::kObjectAtTarget:
      return SceneLeaseReleaseDecision::kRelease;
    case ObjectDisposition::kUnknown:
    case ObjectDisposition::kHoldingObject:
    case ObjectDisposition::kSceneUncertain:
    default:
      return SceneLeaseReleaseDecision::kRetain;
  }
}

SceneLeaseReleaseDecision scene_lease_release_decision(
  const manipulation::task::TaskOutcome & outcome) noexcept
{
  if (!outcome.resources_release_safe()) {
    return SceneLeaseReleaseDecision::kRetain;
  }
  return scene_lease_release_decision(outcome.disposition());
}

SceneLeaseReleaseDecision scene_add_failure_release_decision(
  const std::uint16_t error_code) noexcept
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  switch (error_code) {
    case Result::INVALID_REQUEST:
    case Result::RESOURCE_CONFLICT:
    case Result::LEASE_NOT_FOUND:
    case Result::LEASE_EXPIRED:
    case Result::NOT_OWNER:
    case Result::REVISION_MISMATCH:
    case Result::SESSION_MISMATCH:
    case Result::IDEMPOTENCY_CONFLICT:
    case Result::OPERATION_CAPACITY_EXCEEDED:
    case Result::DEPENDENCY_UNAVAILABLE:
    case Result::RESOURCE_QUARANTINED:
      return SceneLeaseReleaseDecision::kRelease;
    case Result::NONE:
    case Result::BACKEND_FAILED:
    case Result::TIMEOUT:
    case Result::OPERATION_IN_PROGRESS:
    case Result::OPERATION_TICKET_INVALID:
    case Result::RECONCILIATION_REQUIRED:
    case Result::INTERNAL_ERROR:
    default:
      return SceneLeaseReleaseDecision::kRetain;
  }
}

}  // namespace manipulation::workflow
