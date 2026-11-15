// Responsibility: Verifies leases are released only for confirmed safe object dispositions.

#include <cstdint>

#include "gtest/gtest.h"

#include "manipulation_msgs/msg/manipulation_task_result.hpp"
#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"
#include "manipulation/workflow/scene_lease_release_policy.hpp"

namespace manipulation::workflow
{
namespace
{

using TaskResult = manipulation_msgs::msg::ManipulationTaskResult;
using ObjectDisposition = manipulation::task::ObjectDisposition;
using ResourceReleaseDisposition = manipulation::task::ResourceReleaseDisposition;
using TaskErrorCode = manipulation::task::TaskErrorCode;
using TaskOutcome = manipulation::task::TaskOutcome;

TEST(SceneLeaseReleasePolicy, ReleasesOnlyConfirmedDetachedDispositions)
{
  for (const std::uint8_t disposition : {
        TaskResult::SAFE_IDLE,
        TaskResult::OBJECT_AT_SOURCE,
        TaskResult::OBJECT_AT_TARGET})
  {
    EXPECT_EQ(
      scene_lease_release_decision(disposition),
      SceneLeaseReleaseDecision::kRelease);
  }
}

TEST(SceneLeaseReleasePolicy, RetainsUnsafeUnknownAndFutureDispositions)
{
  for (const std::uint8_t disposition : {
        TaskResult::DISPOSITION_UNKNOWN,
        TaskResult::HOLDING_OBJECT,
        TaskResult::SCENE_UNCERTAIN,
        static_cast<std::uint8_t>(255U)})
  {
    EXPECT_EQ(
      scene_lease_release_decision(disposition),
      SceneLeaseReleaseDecision::kRetain);
  }
}

TEST(SceneLeaseReleasePolicy, ClassifiesDomainDispositionsWithoutWireConversion)
{
  for (const auto disposition : {
        ObjectDisposition::kSafeIdle,
        ObjectDisposition::kObjectAtSource,
        ObjectDisposition::kObjectAtTarget})
  {
    EXPECT_EQ(
      scene_lease_release_decision(disposition),
      SceneLeaseReleaseDecision::kRelease);
  }
  for (const auto disposition : {
        ObjectDisposition::kUnknown,
        ObjectDisposition::kHoldingObject,
        ObjectDisposition::kSceneUncertain,
        static_cast<ObjectDisposition>(255U)})
  {
    EXPECT_EQ(
      scene_lease_release_decision(disposition),
      SceneLeaseReleaseDecision::kRetain);
  }
}

TEST(SceneLeaseReleasePolicy, QuarantineOverridesOtherwiseSafeDisposition)
{
  const auto safe_failure = TaskOutcome::make_failure(
    TaskErrorCode::kExecutionFailed, ObjectDisposition::kObjectAtSource);
  const auto quarantined_failure = TaskOutcome::make_failure(
    TaskErrorCode::kExecutionFailed, ObjectDisposition::kObjectAtSource, 0U,
    "accepted command has no terminal result",
    ResourceReleaseDisposition::kQuarantineRequired);

  EXPECT_EQ(
    scene_lease_release_decision(safe_failure),
    SceneLeaseReleaseDecision::kRelease);
  EXPECT_EQ(
    scene_lease_release_decision(quarantined_failure),
    SceneLeaseReleaseDecision::kRetain);
}

TEST(SceneLeaseReleasePolicy, RetainsHoldingAndUncertainTaskOutcomes)
{
  for (const auto disposition : {
        ObjectDisposition::kHoldingObject,
        ObjectDisposition::kSceneUncertain,
        ObjectDisposition::kUnknown})
  {
    const auto outcome = TaskOutcome::make_failure(
      TaskErrorCode::kExecutionFailed, disposition);
    EXPECT_EQ(
      scene_lease_release_decision(outcome),
      SceneLeaseReleaseDecision::kRetain);
  }
}

TEST(SceneLeaseReleasePolicy, ReleasesAddLeaseOnlyAfterPreDispatchRejection)
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  for (const std::uint16_t error_code : {
        Result::INVALID_REQUEST,
        Result::RESOURCE_CONFLICT,
        Result::LEASE_NOT_FOUND,
        Result::LEASE_EXPIRED,
        Result::NOT_OWNER,
        Result::REVISION_MISMATCH,
        Result::SESSION_MISMATCH,
        Result::IDEMPOTENCY_CONFLICT,
        Result::OPERATION_CAPACITY_EXCEEDED,
        Result::DEPENDENCY_UNAVAILABLE,
        Result::RESOURCE_QUARANTINED})
  {
    EXPECT_EQ(
      scene_add_failure_release_decision(error_code),
      SceneLeaseReleaseDecision::kRelease);
  }
}

TEST(SceneLeaseReleasePolicy, RetainsAddLeaseForAmbiguousAndFutureErrors)
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  for (const std::uint16_t error_code : {
        Result::NONE,
        Result::BACKEND_FAILED,
        Result::TIMEOUT,
        Result::OPERATION_IN_PROGRESS,
        Result::OPERATION_TICKET_INVALID,
        Result::RECONCILIATION_REQUIRED,
        Result::INTERNAL_ERROR,
        static_cast<std::uint16_t>(65535U)})
  {
    EXPECT_EQ(
      scene_add_failure_release_decision(error_code),
      SceneLeaseReleaseDecision::kRetain);
  }
}

}  // namespace
}  // namespace manipulation::workflow
