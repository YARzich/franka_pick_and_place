// Responsibility: Verifies complete, stable conversion between domain task state and ROS messages.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/workflow/task_message_adapter.hpp"

namespace manipulation::workflow
{
namespace
{

using manipulation::task::ObjectDisposition;
using manipulation::task::TaskErrorCode;
using manipulation::task::TaskOutcome;
using manipulation::task::TaskPhase;
using manipulation::task::TaskStateSnapshot;
using TaskResult = manipulation_msgs::msg::ManipulationTaskResult;
using TaskState = manipulation_msgs::msg::ManipulationTaskState;

TEST(TaskMessageAdapter, MapsEveryTaskPhaseExplicitly)
{
  const std::vector<std::pair<TaskPhase, std::uint8_t>> cases{
    {TaskPhase::kIdle, TaskState::IDLE},
    {TaskPhase::kValidating, TaskState::VALIDATING},
    {TaskPhase::kWaitingForResources, TaskState::WAITING_FOR_RESOURCES},
    {TaskPhase::kPlanning, TaskState::PLANNING},
    {TaskPhase::kApproaching, TaskState::APPROACHING},
    {TaskPhase::kGrasping, TaskState::GRASPING},
    {TaskPhase::kAttaching, TaskState::ATTACHING},
    {TaskPhase::kLifting, TaskState::LIFTING},
    {TaskPhase::kTransferring, TaskState::TRANSFERRING},
    {TaskPhase::kPlacing, TaskState::PLACING},
    {TaskPhase::kReleasing, TaskState::RELEASING},
    {TaskPhase::kDetaching, TaskState::DETACHING},
    {TaskPhase::kRetreating, TaskState::RETREATING},
    {TaskPhase::kRecovering, TaskState::RECOVERING},
    {TaskPhase::kCompleted, TaskState::COMPLETED},
    {TaskPhase::kCanceled, TaskState::CANCELED},
    {TaskPhase::kFailed, TaskState::FAILED},
    {TaskPhase::kTracking, TaskState::TRACKING},
  };

  for (const auto & [source, expected] : cases) {
    EXPECT_EQ(to_wire_phase(source), expected);
  }
}

TEST(TaskMessageAdapter, MapsEveryErrorCodeExplicitly)
{
  const std::vector<std::pair<TaskErrorCode, std::uint16_t>> cases{
    {TaskErrorCode::kNone, TaskResult::NONE},
    {TaskErrorCode::kCanceled, TaskResult::CANCELED},
    {TaskErrorCode::kInvalidGoal, TaskResult::INVALID_GOAL},
    {TaskErrorCode::kResourceUnavailable, TaskResult::RESOURCE_UNAVAILABLE},
    {TaskErrorCode::kRobotStateStale, TaskResult::ROBOT_STATE_STALE},
    {TaskErrorCode::kControllerUnavailable, TaskResult::CONTROLLER_UNAVAILABLE},
    {TaskErrorCode::kTfError, TaskResult::TF_ERROR},
    {TaskErrorCode::kPerceptionTimeout, TaskResult::PERCEPTION_TIMEOUT},
    {TaskErrorCode::kPlanningFailed, TaskResult::PLANNING_FAILED},
    {TaskErrorCode::kExecutionFailed, TaskResult::EXECUTION_FAILED},
    {TaskErrorCode::kGripperFailed, TaskResult::GRIPPER_FAILED},
    {TaskErrorCode::kSceneConflict, TaskResult::SCENE_CONFLICT},
    {TaskErrorCode::kSceneFailed, TaskResult::SCENE_FAILED},
    {TaskErrorCode::kTimeout, TaskResult::TIMEOUT},
    {TaskErrorCode::kRecoveryFailed, TaskResult::RECOVERY_FAILED},
    {TaskErrorCode::kInternalError, TaskResult::INTERNAL_ERROR},
  };

  for (const auto & [source, expected] : cases) {
    EXPECT_EQ(to_wire_error_code(source), expected);
  }
}

TEST(TaskMessageAdapter, MapsEveryObjectDispositionExplicitly)
{
  const std::vector<std::pair<ObjectDisposition, std::uint8_t>> cases{
    {ObjectDisposition::kUnknown, TaskResult::DISPOSITION_UNKNOWN},
    {ObjectDisposition::kSafeIdle, TaskResult::SAFE_IDLE},
    {ObjectDisposition::kObjectAtSource, TaskResult::OBJECT_AT_SOURCE},
    {ObjectDisposition::kHoldingObject, TaskResult::HOLDING_OBJECT},
    {ObjectDisposition::kObjectAtTarget, TaskResult::OBJECT_AT_TARGET},
    {ObjectDisposition::kSceneUncertain, TaskResult::SCENE_UNCERTAIN},
  };

  for (const auto & [source, expected] : cases) {
    EXPECT_EQ(to_wire_disposition(source), expected);
  }
}

TEST(TaskMessageAdapter, RejectsUnknownEnumRepresentations)
{
  EXPECT_THROW(to_wire_phase(static_cast<TaskPhase>(255U)), std::invalid_argument);
  EXPECT_THROW(to_wire_error_code(static_cast<TaskErrorCode>(65535U)), std::invalid_argument);
  EXPECT_THROW(
    to_wire_disposition(static_cast<ObjectDisposition>(255U)), std::invalid_argument);
}

TEST(TaskMessageAdapter, ConvertsAndNormalizesElapsedDuration)
{
  const auto duration =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::seconds{7} + std::chrono::nanoseconds{123456789});

  const auto message = to_ros_duration(duration);

  EXPECT_EQ(message.sec, 7);
  EXPECT_EQ(message.nanosec, 123456789U);
}

TEST(TaskMessageAdapter, ClampsNonPositiveElapsedDurationToZero)
{
  const auto zero = to_ros_duration(std::chrono::steady_clock::duration::zero());
  const auto negative = to_ros_duration(
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      -std::chrono::milliseconds{1}));

  EXPECT_EQ(zero.sec, 0);
  EXPECT_EQ(zero.nanosec, 0U);
  EXPECT_EQ(negative.sec, 0);
  EXPECT_EQ(negative.nanosec, 0U);
}

TEST(TaskMessageAdapter, SaturatesElapsedDurationAtWireMaximum)
{
  const auto above_maximum =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::seconds{
        static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1LL});

  const auto message = to_ros_duration(above_maximum);

  EXPECT_EQ(message.sec, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(message.nanosec, 999999999U);
}

TEST(TaskMessageAdapter, PreservesMaximumNormalizedNanoseconds)
{
  const auto duration =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::seconds{4} + std::chrono::nanoseconds{999999999});

  const auto message = to_ros_duration(duration);

  EXPECT_EQ(message.sec, 4);
  EXPECT_EQ(message.nanosec, 999999999U);
}

class ProgressPolicyTest : public ::testing::TestWithParam<std::pair<float, float>> {};

TEST_P(ProgressPolicyTest, AppliesDocumentedClampingPolicy)
{
  TaskStateSnapshot snapshot;
  snapshot.progress = GetParam().first;

  EXPECT_FLOAT_EQ(to_ros_message(snapshot).progress, GetParam().second);
}

INSTANTIATE_TEST_SUITE_P(
  ProgressCases,
  ProgressPolicyTest,
  ::testing::Values(
    std::pair{-0.25F, 0.0F},
    std::pair{0.0F, 0.0F},
    std::pair{0.375F, 0.375F},
    std::pair{1.0F, 1.0F},
    std::pair{1.25F, 1.0F},
    std::pair{std::numeric_limits<float>::quiet_NaN(), 0.0F},
    std::pair{-std::numeric_limits<float>::infinity(), 0.0F},
    std::pair{std::numeric_limits<float>::infinity(), 1.0F}));

TEST(TaskMessageAdapter, CopiesSnapshotContractFieldsAndOperatorDetail)
{
  TaskStateSnapshot snapshot;
  snapshot.phase = TaskPhase::kRecovering;
  snapshot.progress = 0.6F;
  snapshot.step_index = 4U;
  snapshot.step_count = 9U;
  snapshot.retry_count = 2U;
  snapshot.recovery_count = 3U;
  snapshot.paused = true;
  snapshot.pause_pending = false;
  snapshot.elapsed = std::chrono::seconds{12};
  snapshot.detail = "retreating after controller rejection";

  const auto message = to_ros_message(snapshot);

  EXPECT_EQ(message.phase, TaskState::RECOVERING);
  EXPECT_FLOAT_EQ(message.progress, 0.6F);
  EXPECT_EQ(message.step_index, 4U);
  EXPECT_EQ(message.step_count, 9U);
  EXPECT_EQ(message.retry_count, 2U);
  EXPECT_EQ(message.recovery_count, 3U);
  EXPECT_TRUE(message.paused);
  EXPECT_FALSE(message.pause_pending);
  EXPECT_EQ(message.elapsed.sec, 12);
  EXPECT_EQ(message.elapsed.nanosec, 0U);
  EXPECT_EQ(message.detail, "retreating after controller rejection");
}

TEST(TaskMessageAdapter, SerializesSuccessOutcome)
{
  const auto outcome = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, "object placed and detached");

  const auto message = to_ros_message(outcome, std::chrono::milliseconds{2500});

  EXPECT_TRUE(message.success);
  EXPECT_EQ(message.error_code, TaskResult::NONE);
  EXPECT_EQ(message.error_message, "object placed and detached");
  EXPECT_EQ(message.final_disposition, TaskResult::OBJECT_AT_TARGET);
  EXPECT_EQ(message.recovery_count, 0U);
  EXPECT_EQ(message.elapsed.sec, 2);
  EXPECT_EQ(message.elapsed.nanosec, 500000000U);
}

TEST(TaskMessageAdapter, SerializesFailureDetailAndRecoveryCount)
{
  const auto outcome = TaskOutcome::make_failure(
    TaskErrorCode::kRecoveryFailed,
    ObjectDisposition::kHoldingObject,
    4U,
    "manual intervention required");

  const auto message = to_ros_message(outcome, std::chrono::seconds{18});

  EXPECT_FALSE(message.success);
  EXPECT_EQ(message.error_code, TaskResult::RECOVERY_FAILED);
  EXPECT_EQ(message.error_message, "manual intervention required");
  EXPECT_EQ(message.final_disposition, TaskResult::HOLDING_OBJECT);
  EXPECT_EQ(message.recovery_count, 4U);
  EXPECT_EQ(message.elapsed.sec, 18);
}

TEST(TaskMessageAdapter, DecodesValidatedSuccessAndFailureResults)
{
  const auto success_message = to_ros_message(
    TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget, 2U, "placed"),
    std::chrono::milliseconds{1250});
  const auto failure_message = to_ros_message(
    TaskOutcome::make_failure(
      TaskErrorCode::kRecoveryFailed, ObjectDisposition::kHoldingObject, 3U,
      "operator required"),
    std::chrono::seconds{9});

  const auto success = decode_task_result(success_message);
  const auto failure = decode_task_result(failure_message);

  ASSERT_TRUE(success.valid()) << success.reason;
  EXPECT_TRUE(success.value->outcome.succeeded());
  EXPECT_EQ(success.value->outcome.disposition(), ObjectDisposition::kObjectAtTarget);
  EXPECT_EQ(success.value->outcome.recovery_count(), 2U);
  EXPECT_EQ(success.value->outcome.detail(), "placed");
  EXPECT_EQ(success.value->elapsed, std::chrono::milliseconds{1250});
  ASSERT_TRUE(failure.valid()) << failure.reason;
  EXPECT_EQ(failure.value->outcome.error_code(), TaskErrorCode::kRecoveryFailed);
  EXPECT_EQ(failure.value->outcome.disposition(), ObjectDisposition::kHoldingObject);
  EXPECT_EQ(failure.value->outcome.recovery_count(), 3U);
  EXPECT_EQ(failure.value->elapsed, std::chrono::seconds{9});
}

TEST(TaskMessageAdapter, RejectsInconsistentOrUnknownTaskResults)
{
  TaskResult message;
  message.success = true;
  message.error_code = TaskResult::PLANNING_FAILED;
  message.final_disposition = TaskResult::OBJECT_AT_SOURCE;
  EXPECT_FALSE(decode_task_result(message).valid());

  message.success = false;
  message.error_code = 65000U;
  EXPECT_FALSE(decode_task_result(message).valid());

  message.error_code = TaskResult::PLANNING_FAILED;
  message.final_disposition = 250U;
  EXPECT_FALSE(decode_task_result(message).valid());

  message.final_disposition = TaskResult::OBJECT_AT_SOURCE;
  message.elapsed.nanosec = 1000000000U;
  EXPECT_FALSE(decode_task_result(message).valid());
}

TEST(TaskMessageAdapter, DecodesValidatedTaskFeedback)
{
  TaskStateSnapshot snapshot;
  snapshot.phase = TaskPhase::kTransferring;
  snapshot.progress = 0.625F;
  snapshot.step_index = 3U;
  snapshot.step_count = 8U;
  snapshot.retry_count = 1U;
  snapshot.recovery_count = 2U;
  snapshot.pause_pending = true;
  snapshot.elapsed = std::chrono::milliseconds{2750};
  snapshot.detail = "controller executing";

  const auto decoded = decode_task_state(to_ros_message(snapshot));

  ASSERT_TRUE(decoded.valid()) << decoded.reason;
  EXPECT_EQ(decoded.value->phase, TaskPhase::kTransferring);
  EXPECT_FLOAT_EQ(decoded.value->progress, 0.625F);
  EXPECT_EQ(decoded.value->step_index, 3U);
  EXPECT_EQ(decoded.value->step_count, 8U);
  EXPECT_EQ(decoded.value->retry_count, 1U);
  EXPECT_EQ(decoded.value->recovery_count, 2U);
  EXPECT_TRUE(decoded.value->pause_pending);
  EXPECT_EQ(decoded.value->elapsed, std::chrono::milliseconds{2750});
  EXPECT_EQ(decoded.value->detail, "controller executing");
}

TEST(TaskMessageAdapter, RejectsMalformedTaskFeedback)
{
  TaskState message;
  message.phase = 250U;
  EXPECT_FALSE(decode_task_state(message).valid());

  message.phase = TaskState::PLANNING;
  message.progress = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(decode_task_state(message).valid());

  message.progress = 0.5F;
  message.elapsed.sec = -1;
  EXPECT_FALSE(decode_task_state(message).valid());
}

}  // namespace
}  // namespace manipulation::workflow
