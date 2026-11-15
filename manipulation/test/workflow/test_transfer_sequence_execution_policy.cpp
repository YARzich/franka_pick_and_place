// Responsibility: Verifies pure fail-closed and lifecycle normalization of sequence evidence.

#include <chrono>
#include <optional>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "detail/transfer_sequence_execution_policy.hpp"
#include "manipulation/task/task_status.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using namespace std::chrono_literals;
using ObjectDisposition = manipulation::task::ObjectDisposition;
using ResourceReleaseDisposition = manipulation::task::ResourceReleaseDisposition;
using TaskErrorCode = manipulation::task::TaskErrorCode;
using TaskOutcome = manipulation::task::TaskOutcome;
constexpr char kActionLabel[] = "SequenceA";

TransferSequenceExecution one_step_success()
{
  const auto outcome = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, "step completed");
  return {
    TransferSequenceResult{outcome, 1U, std::nullopt, {outcome}},
    {7ms}};
}

TEST(TransferSequenceExecutionPolicy, PreservesValidSuccessfulEvidence)
{
  auto result = apply_transfer_sequence_execution_policy(
    one_step_success(), 1U, TaskStopCause::kNone, kActionLabel);

  EXPECT_FALSE(result.contract_error.has_value());
  EXPECT_TRUE(result.execution.result.outcome.succeeded());
  EXPECT_EQ(result.execution.result.completed_step_count, 1U);
  ASSERT_EQ(result.execution.result.step_outcomes.size(), 1U);
  EXPECT_EQ(
    result.execution.result.step_outcomes.front().disposition(),
    ObjectDisposition::kObjectAtTarget);
  ASSERT_EQ(result.execution.step_elapsed.size(), 1U);
  EXPECT_EQ(result.execution.step_elapsed.front(), 7ms);
}

TEST(TransferSequenceExecutionPolicy, InvalidEvidenceFailsClosedWithoutSteps)
{
  auto execution = one_step_success();
  execution.step_elapsed.clear();

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kNone, kActionLabel);

  ASSERT_TRUE(result.contract_error.has_value());
  EXPECT_NE(result.contract_error->find("sizes differ"), std::string::npos);
  EXPECT_EQ(result.execution.result.outcome.error_code(), TaskErrorCode::kInternalError);
  EXPECT_EQ(result.execution.result.outcome.disposition(), ObjectDisposition::kUnknown);
  EXPECT_EQ(result.execution.result.completed_step_count, 0U);
  EXPECT_FALSE(result.execution.result.failed_step_index.has_value());
  EXPECT_TRUE(result.execution.result.step_outcomes.empty());
  EXPECT_TRUE(result.execution.step_elapsed.empty());
}

TEST(TransferSequenceExecutionPolicy, UsesOwningFacadeLabelInDiagnostics)
{
  auto execution = one_step_success();
  execution.step_elapsed.clear();

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kDeadlineExceeded,
    "SequenceB");

  ASSERT_TRUE(result.contract_error.has_value());
  EXPECT_NE(
    result.execution.result.outcome.detail().find(
      "SequenceB exceeded"),
    std::string::npos);
  EXPECT_NE(
    result.execution.result.outcome.detail().find(
      "invalid SequenceB backend result"),
    std::string::npos);
  EXPECT_EQ(
    result.execution.result.outcome.detail().find("SequenceA"),
    std::string::npos);
}

TEST(TransferSequenceExecutionPolicy, AcceptedCancelOwnsMalformedEvidence)
{
  auto execution = one_step_success();
  execution.step_elapsed.clear();

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kClientCanceled, kActionLabel);

  ASSERT_TRUE(result.contract_error.has_value());
  EXPECT_EQ(result.execution.result.outcome.error_code(), TaskErrorCode::kCanceled);
  EXPECT_EQ(result.execution.result.outcome.disposition(), ObjectDisposition::kUnknown);
  EXPECT_NE(
    result.execution.result.outcome.detail().find("canceled by action client"),
    std::string::npos);
  EXPECT_NE(
    result.execution.result.outcome.detail().find("invalid SequenceA"),
    std::string::npos);
  EXPECT_TRUE(result.execution.result.step_outcomes.empty());
}

TEST(TransferSequenceExecutionPolicy, MalformedEvidenceRetainsAnyQuarantineRequirement)
{
  const auto aggregate = TaskOutcome::make_failure(
    TaskErrorCode::kExecutionFailed, ObjectDisposition::kObjectAtSource);
  const auto quarantined_step = TaskOutcome::make_failure(
    TaskErrorCode::kExecutionFailed, ObjectDisposition::kObjectAtSource, 0U,
    "accepted command outcome is unknown",
    ResourceReleaseDisposition::kQuarantineRequired);
  TransferSequenceExecution execution{
    TransferSequenceResult{aggregate, 0U, 0U, {quarantined_step}},
    {4ms}};

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kNone, kActionLabel);

  ASSERT_TRUE(result.contract_error.has_value());
  EXPECT_FALSE(result.execution.result.outcome.resources_release_safe());
  EXPECT_EQ(
    result.execution.result.outcome.resource_release(),
    ResourceReleaseDisposition::kQuarantineRequired);
  EXPECT_EQ(result.execution.result.outcome.error_code(), TaskErrorCode::kInternalError);
  EXPECT_TRUE(result.execution.result.step_outcomes.empty());
}

TEST(TransferSequenceExecutionPolicy, OuterStopNormalizationPreservesQuarantine)
{
  for (const auto [cause, expected_error] : {
        std::pair{TaskStopCause::kClientCanceled, TaskErrorCode::kCanceled},
        std::pair{TaskStopCause::kDeadlineExceeded, TaskErrorCode::kTimeout},
        std::pair{TaskStopCause::kShutdown, TaskErrorCode::kInternalError}})
  {
    auto quarantined = TaskOutcome::make_failure(
      TaskErrorCode::kExecutionFailed, ObjectDisposition::kHoldingObject, 1U,
      "accepted command has no terminal result",
      ResourceReleaseDisposition::kQuarantineRequired);
    TransferSequenceExecution execution{
      TransferSequenceResult{std::move(quarantined), 0U, std::nullopt, {}}, {}};

    auto result = apply_transfer_sequence_execution_policy(
      std::move(execution), 1U, cause, kActionLabel);

    EXPECT_FALSE(result.contract_error.has_value());
    EXPECT_EQ(result.execution.result.outcome.error_code(), expected_error);
    EXPECT_EQ(
      result.execution.result.outcome.disposition(),
      ObjectDisposition::kHoldingObject);
    EXPECT_EQ(
      result.execution.result.outcome.resource_release(),
      ResourceReleaseDisposition::kQuarantineRequired);
  }
}

TEST(TransferSequenceExecutionPolicy, CancelPreservesAndAlignsCleanupEvidence)
{
  const auto canceled = TaskOutcome::make_failure(
    TaskErrorCode::kCanceled, ObjectDisposition::kSceneUncertain, 2U,
    "sequence scene cleanup incomplete: retained=1",
    ResourceReleaseDisposition::kQuarantineRequired);
  TransferSequenceExecution execution{
    TransferSequenceResult{canceled, 0U, 0U, {canceled}},
    {11ms}};

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kClientCanceled, kActionLabel);

  EXPECT_FALSE(result.contract_error.has_value());
  const auto & aggregate = result.execution.result.outcome;
  EXPECT_EQ(aggregate.error_code(), TaskErrorCode::kCanceled);
  EXPECT_EQ(aggregate.disposition(), ObjectDisposition::kSceneUncertain);
  EXPECT_EQ(aggregate.recovery_count(), 2U);
  EXPECT_EQ(
    aggregate.resource_release(),
    ResourceReleaseDisposition::kQuarantineRequired);
  EXPECT_NE(aggregate.detail().find("cleanup incomplete"), std::string::npos);
  ASSERT_EQ(result.execution.result.step_outcomes.size(), 1U);
  const auto & failed_step = result.execution.result.step_outcomes.front();
  EXPECT_EQ(failed_step.error_code(), aggregate.error_code());
  EXPECT_EQ(failed_step.disposition(), aggregate.disposition());
  EXPECT_EQ(failed_step.recovery_count(), aggregate.recovery_count());
  EXPECT_EQ(failed_step.resource_release(), aggregate.resource_release());
  EXPECT_EQ(failed_step.detail(), aggregate.detail());
}

TEST(TransferSequenceExecutionPolicy, DeadlineAlignsFailedStepWithTimeout)
{
  const auto canceled = TaskOutcome::make_failure(
    TaskErrorCode::kCanceled, ObjectDisposition::kSceneUncertain, 0U,
    "inner transfer canceled; scene cleanup incomplete");
  TransferSequenceExecution execution{
    TransferSequenceResult{canceled, 0U, 0U, {canceled}},
    {19ms}};

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kDeadlineExceeded, kActionLabel);

  EXPECT_FALSE(result.contract_error.has_value());
  EXPECT_EQ(result.execution.result.outcome.error_code(), TaskErrorCode::kTimeout);
  EXPECT_EQ(result.execution.result.outcome.disposition(), ObjectDisposition::kSceneUncertain);
  EXPECT_NE(
    result.execution.result.outcome.detail().find("cleanup incomplete"),
    std::string::npos);
  ASSERT_EQ(result.execution.result.step_outcomes.size(), 1U);
  EXPECT_EQ(
    result.execution.result.step_outcomes.front().error_code(),
    TaskErrorCode::kTimeout);
  EXPECT_EQ(result.execution.step_elapsed.front(), 19ms);
}

TEST(TransferSequenceExecutionPolicy, BackendCannotInventClientCancel)
{
  const auto canceled = TaskOutcome::make_failure(
    TaskErrorCode::kCanceled, ObjectDisposition::kObjectAtSource, 0U,
    "backend-local cancel");
  TransferSequenceExecution execution{
    TransferSequenceResult{canceled, 0U, std::nullopt, {}}, {}};

  auto result = apply_transfer_sequence_execution_policy(
    std::move(execution), 1U, TaskStopCause::kNone, kActionLabel);

  EXPECT_FALSE(result.contract_error.has_value());
  EXPECT_EQ(result.execution.result.outcome.error_code(), TaskErrorCode::kInternalError);
  EXPECT_EQ(result.execution.result.outcome.disposition(), ObjectDisposition::kObjectAtSource);
  EXPECT_NE(
    result.execution.result.outcome.detail().find("without an accepted client cancel"),
    std::string::npos);
}

}  // namespace
}  // namespace manipulation::workflow::detail
