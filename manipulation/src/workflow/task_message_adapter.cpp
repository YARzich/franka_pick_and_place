// Responsibility: Serializes task phases, errors, progress and outcomes into ROS wire values.

#include "manipulation/workflow/task_message_adapter.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation::workflow
{
namespace
{

using TaskResult = manipulation_msgs::msg::ManipulationTaskResult;
using TaskState = manipulation_msgs::msg::ManipulationTaskState;

constexpr std::int32_t kMaxDurationSeconds = std::numeric_limits<std::int32_t>::max();
constexpr std::uint32_t kMaxDurationNanoseconds = 999999999U;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

float normalize_progress(const float progress) noexcept
{
  if (std::isnan(progress) || progress <= 0.0F) {
    return 0.0F;
  }
  if (progress >= 1.0F) {
    return 1.0F;
  }
  return progress;
}

std::optional<manipulation::task::TaskErrorCode> from_wire_error_code(
  const std::uint16_t error_code) noexcept
{
  using Error = manipulation::task::TaskErrorCode;
  switch (error_code) {
    case TaskResult::NONE: return Error::kNone;
    case TaskResult::CANCELED: return Error::kCanceled;
    case TaskResult::INVALID_GOAL: return Error::kInvalidGoal;
    case TaskResult::RESOURCE_UNAVAILABLE: return Error::kResourceUnavailable;
    case TaskResult::ROBOT_STATE_STALE: return Error::kRobotStateStale;
    case TaskResult::CONTROLLER_UNAVAILABLE: return Error::kControllerUnavailable;
    case TaskResult::TF_ERROR: return Error::kTfError;
    case TaskResult::PERCEPTION_TIMEOUT: return Error::kPerceptionTimeout;
    case TaskResult::PLANNING_FAILED: return Error::kPlanningFailed;
    case TaskResult::EXECUTION_FAILED: return Error::kExecutionFailed;
    case TaskResult::GRIPPER_FAILED: return Error::kGripperFailed;
    case TaskResult::SCENE_CONFLICT: return Error::kSceneConflict;
    case TaskResult::SCENE_FAILED: return Error::kSceneFailed;
    case TaskResult::TIMEOUT: return Error::kTimeout;
    case TaskResult::RECOVERY_FAILED: return Error::kRecoveryFailed;
    case TaskResult::INTERNAL_ERROR: return Error::kInternalError;
    default: return std::nullopt;
  }
}

std::optional<manipulation::task::ObjectDisposition> from_wire_disposition(
  const std::uint8_t disposition) noexcept
{
  using Disposition = manipulation::task::ObjectDisposition;
  switch (disposition) {
    case TaskResult::DISPOSITION_UNKNOWN: return Disposition::kUnknown;
    case TaskResult::SAFE_IDLE: return Disposition::kSafeIdle;
    case TaskResult::OBJECT_AT_SOURCE: return Disposition::kObjectAtSource;
    case TaskResult::HOLDING_OBJECT: return Disposition::kHoldingObject;
    case TaskResult::OBJECT_AT_TARGET: return Disposition::kObjectAtTarget;
    case TaskResult::SCENE_UNCERTAIN: return Disposition::kSceneUncertain;
    default: return std::nullopt;
  }
}

std::optional<manipulation::task::TaskPhase> from_wire_phase(
  const std::uint8_t phase) noexcept
{
  using Phase = manipulation::task::TaskPhase;
  switch (phase) {
    case TaskState::IDLE: return Phase::kIdle;
    case TaskState::VALIDATING: return Phase::kValidating;
    case TaskState::WAITING_FOR_RESOURCES: return Phase::kWaitingForResources;
    case TaskState::PLANNING: return Phase::kPlanning;
    case TaskState::APPROACHING: return Phase::kApproaching;
    case TaskState::GRASPING: return Phase::kGrasping;
    case TaskState::ATTACHING: return Phase::kAttaching;
    case TaskState::LIFTING: return Phase::kLifting;
    case TaskState::TRANSFERRING: return Phase::kTransferring;
    case TaskState::PLACING: return Phase::kPlacing;
    case TaskState::RELEASING: return Phase::kReleasing;
    case TaskState::DETACHING: return Phase::kDetaching;
    case TaskState::RETREATING: return Phase::kRetreating;
    case TaskState::RECOVERING: return Phase::kRecovering;
    case TaskState::COMPLETED: return Phase::kCompleted;
    case TaskState::CANCELED: return Phase::kCanceled;
    case TaskState::FAILED: return Phase::kFailed;
    case TaskState::TRACKING: return Phase::kTracking;
    default: return std::nullopt;
  }
}

std::optional<std::chrono::steady_clock::duration> from_ros_duration(
  const builtin_interfaces::msg::Duration & duration) noexcept
{
  if (duration.sec < 0 || duration.nanosec >= 1000000000U) {
    return std::nullopt;
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::seconds{duration.sec} + std::chrono::nanoseconds{duration.nanosec});
}

}  // namespace

std::uint8_t to_wire_phase(const manipulation::task::TaskPhase phase)
{
  switch (phase) {
    case manipulation::task::TaskPhase::kIdle:
      return TaskState::IDLE;
    case manipulation::task::TaskPhase::kValidating:
      return TaskState::VALIDATING;
    case manipulation::task::TaskPhase::kWaitingForResources:
      return TaskState::WAITING_FOR_RESOURCES;
    case manipulation::task::TaskPhase::kPlanning:
      return TaskState::PLANNING;
    case manipulation::task::TaskPhase::kApproaching:
      return TaskState::APPROACHING;
    case manipulation::task::TaskPhase::kGrasping:
      return TaskState::GRASPING;
    case manipulation::task::TaskPhase::kAttaching:
      return TaskState::ATTACHING;
    case manipulation::task::TaskPhase::kLifting:
      return TaskState::LIFTING;
    case manipulation::task::TaskPhase::kTransferring:
      return TaskState::TRANSFERRING;
    case manipulation::task::TaskPhase::kPlacing:
      return TaskState::PLACING;
    case manipulation::task::TaskPhase::kReleasing:
      return TaskState::RELEASING;
    case manipulation::task::TaskPhase::kDetaching:
      return TaskState::DETACHING;
    case manipulation::task::TaskPhase::kRetreating:
      return TaskState::RETREATING;
    case manipulation::task::TaskPhase::kRecovering:
      return TaskState::RECOVERING;
    case manipulation::task::TaskPhase::kCompleted:
      return TaskState::COMPLETED;
    case manipulation::task::TaskPhase::kCanceled:
      return TaskState::CANCELED;
    case manipulation::task::TaskPhase::kFailed:
      return TaskState::FAILED;
    case manipulation::task::TaskPhase::kTracking:
      return TaskState::TRACKING;
  }
  throw std::invalid_argument("unknown TaskPhase value");
}

std::uint16_t to_wire_error_code(const manipulation::task::TaskErrorCode error_code)
{
  switch (error_code) {
    case manipulation::task::TaskErrorCode::kNone:
      return TaskResult::NONE;
    case manipulation::task::TaskErrorCode::kCanceled:
      return TaskResult::CANCELED;
    case manipulation::task::TaskErrorCode::kInvalidGoal:
      return TaskResult::INVALID_GOAL;
    case manipulation::task::TaskErrorCode::kResourceUnavailable:
      return TaskResult::RESOURCE_UNAVAILABLE;
    case manipulation::task::TaskErrorCode::kRobotStateStale:
      return TaskResult::ROBOT_STATE_STALE;
    case manipulation::task::TaskErrorCode::kControllerUnavailable:
      return TaskResult::CONTROLLER_UNAVAILABLE;
    case manipulation::task::TaskErrorCode::kTfError:
      return TaskResult::TF_ERROR;
    case manipulation::task::TaskErrorCode::kPerceptionTimeout:
      return TaskResult::PERCEPTION_TIMEOUT;
    case manipulation::task::TaskErrorCode::kPlanningFailed:
      return TaskResult::PLANNING_FAILED;
    case manipulation::task::TaskErrorCode::kExecutionFailed:
      return TaskResult::EXECUTION_FAILED;
    case manipulation::task::TaskErrorCode::kGripperFailed:
      return TaskResult::GRIPPER_FAILED;
    case manipulation::task::TaskErrorCode::kSceneConflict:
      return TaskResult::SCENE_CONFLICT;
    case manipulation::task::TaskErrorCode::kSceneFailed:
      return TaskResult::SCENE_FAILED;
    case manipulation::task::TaskErrorCode::kTimeout:
      return TaskResult::TIMEOUT;
    case manipulation::task::TaskErrorCode::kRecoveryFailed:
      return TaskResult::RECOVERY_FAILED;
    case manipulation::task::TaskErrorCode::kInternalError:
      return TaskResult::INTERNAL_ERROR;
  }
  throw std::invalid_argument("unknown TaskErrorCode value");
}

std::uint8_t to_wire_disposition(
  const manipulation::task::ObjectDisposition disposition)
{
  switch (disposition) {
    case manipulation::task::ObjectDisposition::kUnknown:
      return TaskResult::DISPOSITION_UNKNOWN;
    case manipulation::task::ObjectDisposition::kSafeIdle:
      return TaskResult::SAFE_IDLE;
    case manipulation::task::ObjectDisposition::kObjectAtSource:
      return TaskResult::OBJECT_AT_SOURCE;
    case manipulation::task::ObjectDisposition::kHoldingObject:
      return TaskResult::HOLDING_OBJECT;
    case manipulation::task::ObjectDisposition::kObjectAtTarget:
      return TaskResult::OBJECT_AT_TARGET;
    case manipulation::task::ObjectDisposition::kSceneUncertain:
      return TaskResult::SCENE_UNCERTAIN;
  }
  throw std::invalid_argument("unknown ObjectDisposition value");
}

builtin_interfaces::msg::Duration to_ros_duration(
  const std::chrono::steady_clock::duration elapsed) noexcept
{
  builtin_interfaces::msg::Duration result;
  if (elapsed <= std::chrono::steady_clock::duration::zero()) {
    return result;
  }

  const auto maximum =
    std::chrono::seconds{kMaxDurationSeconds} +
  std::chrono::nanoseconds{kMaxDurationNanoseconds};
  const auto maximum_steady =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(maximum);
  if (elapsed >= maximum_steady) {
    result.sec = kMaxDurationSeconds;
    result.nanosec = kMaxDurationNanoseconds;
    return result;
  }

  // Conversion is now bounded by the ROS message range, so nanoseconds fit in int64.
  const auto elapsed_nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  result.sec = static_cast<std::int32_t>(elapsed_nanoseconds / kNanosecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(elapsed_nanoseconds % kNanosecondsPerSecond);
  return result;
}

TaskState to_ros_message(const manipulation::task::TaskStateSnapshot & snapshot)
{
  TaskState result;
  result.phase = to_wire_phase(snapshot.phase);
  result.progress = normalize_progress(snapshot.progress);
  result.step_index = snapshot.step_index;
  result.step_count = snapshot.step_count;
  result.retry_count = snapshot.retry_count;
  result.recovery_count = snapshot.recovery_count;
  result.paused = snapshot.paused;
  result.pause_pending = snapshot.pause_pending;
  result.elapsed = to_ros_duration(snapshot.elapsed);
  result.detail = snapshot.detail;
  return result;
}

TaskResult to_ros_message(
  const manipulation::task::TaskOutcome & outcome,
  const std::chrono::steady_clock::duration elapsed)
{
  TaskResult result;
  result.success = outcome.succeeded();
  result.error_code = to_wire_error_code(outcome.error_code());
  result.error_message = outcome.detail();
  result.final_disposition = to_wire_disposition(outcome.disposition());
  result.recovery_count = outcome.recovery_count();
  result.elapsed = to_ros_duration(elapsed);
  return result;
}

TaskResultDecodeResult decode_task_result(const TaskResult & message)
{
  const auto error = from_wire_error_code(message.error_code);
  if (!error) {
    return {std::nullopt, "unknown task error code"};
  }
  const auto disposition = from_wire_disposition(message.final_disposition);
  if (!disposition) {
    return {std::nullopt, "unknown object disposition"};
  }
  if (message.success != (*error == manipulation::task::TaskErrorCode::kNone)) {
    return {std::nullopt, "success and error_code are inconsistent"};
  }
  const auto elapsed = from_ros_duration(message.elapsed);
  if (!elapsed) {
    return {std::nullopt, "elapsed duration is invalid"};
  }
  auto outcome = message.success ?
    manipulation::task::TaskOutcome::make_success(
    *disposition, message.recovery_count, message.error_message) :
    manipulation::task::TaskOutcome::make_failure(
    *error, *disposition, message.recovery_count, message.error_message);
  return {
    DecodedTaskResult{std::move(outcome), *elapsed}, {}};
}

TaskStateDecodeResult decode_task_state(const TaskState & message)
{
  const auto phase = from_wire_phase(message.phase);
  if (!phase) {
    return {std::nullopt, "unknown task phase"};
  }
  if (!std::isfinite(message.progress) || message.progress < 0.0F ||
    message.progress > 1.0F)
  {
    return {std::nullopt, "task progress is outside [0, 1]"};
  }
  const auto elapsed = from_ros_duration(message.elapsed);
  if (!elapsed) {
    return {std::nullopt, "task elapsed duration is invalid"};
  }

  manipulation::task::TaskStateSnapshot state;
  state.phase = *phase;
  state.progress = message.progress;
  state.step_index = message.step_index;
  state.step_count = message.step_count;
  state.retry_count = message.retry_count;
  state.recovery_count = message.recovery_count;
  state.paused = message.paused;
  state.pause_pending = message.pause_pending;
  state.elapsed = *elapsed;
  state.detail = message.detail;
  return {std::move(state), {}};
}

}  // namespace manipulation::workflow
