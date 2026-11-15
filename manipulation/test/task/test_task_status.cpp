// Responsibility: Verifies task phase, outcome, error, and object-disposition invariants.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "manipulation/task/task_status.hpp"

namespace
{

using manipulation::task::ObjectDisposition;
using manipulation::task::ResourceReleaseDisposition;
using manipulation::task::TaskErrorCode;
using manipulation::task::TaskOutcome;
using manipulation::task::TaskPhase;

TEST(TaskStatus, ClassifiesOnlyTerminalPhasesAsTerminal)
{
  EXPECT_FALSE(manipulation::task::is_terminal(TaskPhase::kIdle));
  EXPECT_FALSE(manipulation::task::is_terminal(TaskPhase::kRecovering));
  EXPECT_TRUE(manipulation::task::is_terminal(TaskPhase::kCompleted));
  EXPECT_TRUE(manipulation::task::is_terminal(TaskPhase::kCanceled));
  EXPECT_TRUE(manipulation::task::is_terminal(TaskPhase::kFailed));
}

TEST(TaskStatus, SuccessHasNoErrorAndPreservesDisposition)
{
  const auto outcome = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, "placement complete");

  EXPECT_TRUE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), TaskErrorCode::kNone);
  EXPECT_EQ(outcome.disposition(), ObjectDisposition::kObjectAtTarget);
  EXPECT_EQ(outcome.recovery_count(), 0U);
  EXPECT_EQ(outcome.detail(), "placement complete");
  EXPECT_TRUE(outcome.resources_release_safe());
}

TEST(TaskStatus, SuccessfulRecoveryPreservesAttemptCount)
{
  const auto outcome = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, 2U, "placement recovered");

  EXPECT_TRUE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), TaskErrorCode::kNone);
  EXPECT_EQ(outcome.recovery_count(), 2U);
}

TEST(TaskStatus, FailurePreservesMachineReadableContext)
{
  const auto outcome = TaskOutcome::make_failure(
    TaskErrorCode::kRecoveryFailed, ObjectDisposition::kHoldingObject,
    2U, "operator intervention required");

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), TaskErrorCode::kRecoveryFailed);
  EXPECT_EQ(outcome.disposition(), ObjectDisposition::kHoldingObject);
  EXPECT_EQ(outcome.recovery_count(), 2U);
  EXPECT_EQ(outcome.detail(), "operator intervention required");
}

TEST(TaskStatus, FailureRejectsSuccessErrorCode)
{
  EXPECT_THROW(
    TaskOutcome::make_failure(
      TaskErrorCode::kNone, ObjectDisposition::kUnknown),
    std::invalid_argument);
}

TEST(TaskStatus, FailurePreservesExplicitResourceQuarantineRequirement)
{
  const auto outcome = TaskOutcome::make_failure(
    TaskErrorCode::kTimeout, ObjectDisposition::kHoldingObject, 1U,
    "controller result is unresolved",
    ResourceReleaseDisposition::kQuarantineRequired);

  EXPECT_FALSE(outcome.resources_release_safe());
  EXPECT_EQ(
    outcome.resource_release(),
    ResourceReleaseDisposition::kQuarantineRequired);
}

TEST(TaskStatus, IdentifiesDispositionsThatNeedAttention)
{
  EXPECT_TRUE(manipulation::task::requires_attention(ObjectDisposition::kUnknown));
  EXPECT_TRUE(manipulation::task::requires_attention(ObjectDisposition::kHoldingObject));
  EXPECT_TRUE(manipulation::task::requires_attention(ObjectDisposition::kSceneUncertain));
  EXPECT_FALSE(manipulation::task::requires_attention(ObjectDisposition::kSafeIdle));
  EXPECT_FALSE(manipulation::task::requires_attention(ObjectDisposition::kObjectAtSource));
  EXPECT_FALSE(manipulation::task::requires_attention(ObjectDisposition::kObjectAtTarget));
}

}  // namespace
