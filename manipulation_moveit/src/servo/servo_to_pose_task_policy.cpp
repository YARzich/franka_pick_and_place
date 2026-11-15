// Responsibility: Maps ServoToPose state-machine evidence to task feedback and results.

#include "servo_to_pose_task_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace manipulation::moveit::servo_to_pose_detail
{
namespace
{

using ErrorCode = manipulation::task::TaskErrorCode;
using HaltPhase = manipulation::servo::ServoToPoseHaltPhase;
using ServoState = manipulation::servo::ServoToPoseState;
using StopCause = manipulation::servo::ServoToPoseStopCause;
using TerminalReason = manipulation::servo::ServoToPoseTerminalReason;

bool coherent_tracking_error(
  const MoveItServoObservationResult & observation) noexcept
{
  const auto & sample = observation.observation.pose_sample;
  return observation.status.succeeded() && !observation.observation.ros_time_moved_backwards &&
         sample.available && sample.fresh && sample.valid && sample.sequence != 0U &&
         std::isfinite(sample.position_error_m) && sample.position_error_m >= 0.0 &&
         std::isfinite(sample.orientation_error_rad) && sample.orientation_error_rad >= 0.0;
}

manipulation::task::TaskPhase feedback_phase(
  const manipulation::servo::ServoToPoseStepResult & step) noexcept
{
  switch (step.state) {
    case ServoState::kAwaitingResume:
      return manipulation::task::TaskPhase::kValidating;
    case ServoState::kTracking:
    case ServoState::kPaused:
      return manipulation::task::TaskPhase::kTracking;
    case ServoState::kHalting:
      return manipulation::task::TaskPhase::kRecovering;
    case ServoState::kTerminal:
      switch (step.terminal_reason) {
        case TerminalReason::kSucceeded:
          return manipulation::task::TaskPhase::kCompleted;
        case TerminalReason::kCanceled:
          return manipulation::task::TaskPhase::kCanceled;
        case TerminalReason::kNone:
        case TerminalReason::kGoalTimeout:
        case TerminalReason::kPoseSampleStale:
        case TerminalReason::kPoseSampleInvalid:
        case TerminalReason::kBackendSafetyStop:
        case TerminalReason::kHaltTimeout:
        case TerminalReason::kFinalPoseOutsideTolerance:
        case TerminalReason::kFinalPoseSampleStale:
        case TerminalReason::kFinalPoseSampleInvalid:
          return manipulation::task::TaskPhase::kFailed;
      }
  }
  return manipulation::task::TaskPhase::kFailed;
}

std::string feedback_detail(const manipulation::servo::ServoToPoseStepResult & step)
{
  switch (step.state) {
    case ServoState::kAwaitingResume:
      return "preparing MoveIt Servo pose tracking";
    case ServoState::kTracking:
      return "tracking the fixed ServoToPose target";
    case ServoState::kHalting: {
        switch (step.halt_phase) {
          case HaltPhase::kWaitingForLowVelocity:
            return "target publication stopped; confirming low joint velocity";
          case HaltPhase::kAwaitingBackendPause:
            return "low joint velocity confirmed; waiting for Servo pause";
          case HaltPhase::kVerifyingFinalPose:
            return "Servo is paused; verifying the stopped TCP at the final pose";
          case HaltPhase::kNone:
            return "ServoToPose halt processing is active";
        }
        return "unknown ServoToPose halt phase";
      }
    case ServoState::kPaused:
      return "ServoToPose is safely paused";
    case ServoState::kTerminal:
      return "ServoToPose reached a terminal state";
  }
  return "unknown ServoToPose state";
}

}  // namespace

Clock::time_point bounded_deadline(
  const Clock::time_point absolute_deadline,
  const Clock::duration operation_timeout) noexcept
{
  const auto now = Clock::now();
  if (now >= absolute_deadline) {
    return now;
  }
  const auto maximum = Clock::time_point::max() - now;
  const auto requested = operation_timeout >= maximum ?
    Clock::time_point::max() : now + operation_timeout;
  return std::min(absolute_deadline, requested);
}

Clock::duration nonnegative_elapsed(const Clock::time_point started_at) noexcept
{
  const auto now = Clock::now();
  return now > started_at ? now - started_at : Clock::duration::zero();
}

ErrorCode setup_error_code(
  const MoveItServoRosError error, const bool target_preparation) noexcept
{
  if (target_preparation) {
    switch (error) {
      case MoveItServoRosError::kTimeDiscontinuity:
      case MoveItServoRosError::kUnavailable:
      case MoveItServoRosError::kTimeout:
      case MoveItServoRosError::kInvalidData:
      case MoveItServoRosError::kRejected:
        return ErrorCode::kTfError;
      case MoveItServoRosError::kInternalError:
      case MoveItServoRosError::kNone:
        return ErrorCode::kInternalError;
    }
  }
  switch (error) {
    case MoveItServoRosError::kUnavailable:
    case MoveItServoRosError::kTimeout:
    case MoveItServoRosError::kRejected:
      return ErrorCode::kControllerUnavailable;
    case MoveItServoRosError::kTimeDiscontinuity:
      return ErrorCode::kRobotStateStale;
    case MoveItServoRosError::kInvalidData:
      return ErrorCode::kExecutionFailed;
    case MoveItServoRosError::kInternalError:
    case MoveItServoRosError::kNone:
      return ErrorCode::kInternalError;
  }
  return ErrorCode::kInternalError;
}

PendingFailure observation_failure(const MoveItServoRosResult & status)
{
  ErrorCode code{ErrorCode::kRobotStateStale};
  if (status.error() == MoveItServoRosError::kInvalidData) {
    code = ErrorCode::kTfError;
  } else if (status.error() == MoveItServoRosError::kInternalError) {
    code = ErrorCode::kInternalError;
  }
  return {code, "Servo observation failed: " + status.detail()};
}

bool startup_feedback_ready(const MoveItServoObservationResult & observation) noexcept
{
  const auto & value = observation.observation;
  return observation.status.succeeded() && !value.ros_time_moved_backwards &&
         value.backend_state == manipulation::servo::ServoBackendState::kRunning &&
         value.status.available && value.status.fresh && value.status.valid &&
         value.pose_sample.available && value.pose_sample.fresh && value.pose_sample.valid &&
         value.velocity_sample.available && value.velocity_sample.fresh &&
         value.velocity_sample.valid;
}

void emit_feedback(
  const ServoToPoseTask::FeedbackCallback & callback,
  const ServoToPoseTaskFeedback & feedback) noexcept
{
  try {
    callback(feedback);
  } catch (...) {
    // Feedback is observational and must never bypass the physical stop path.
  }
}

ServoToPoseTaskFeedback make_feedback(
  const manipulation::servo::ServoToPoseStepResult & step,
  const MoveItServoObservationResult & observation,
  const ServoToPoseTaskControl::Snapshot & control,
  const Clock::duration elapsed)
{
  ServoToPoseTaskFeedback feedback;
  feedback.state.phase = feedback_phase(step);
  feedback.state.progress = step.terminal_reason == TerminalReason::kSucceeded ? 1.0F : 0.0F;
  feedback.state.step_count = 1U;
  feedback.state.paused = step.state == ServoState::kPaused;
  feedback.state.pause_pending = control.pause_requested && !feedback.state.paused &&
    !control.cancel_requested;
  feedback.state.elapsed = elapsed;
  feedback.state.detail = feedback_detail(step);
  feedback.tracking_error_valid = coherent_tracking_error(observation);
  if (feedback.tracking_error_valid) {
    feedback.linear_error_m = observation.observation.pose_sample.position_error_m;
    feedback.angular_error_rad = observation.observation.pose_sample.orientation_error_rad;
  }
  return feedback;
}

ErrorCode terminal_error_code(
  const manipulation::servo::ServoToPoseStepResult & step,
  const std::optional<PendingFailure> & pending_failure) noexcept
{
  if (step.terminal_reason == TerminalReason::kHaltTimeout) {
    return ErrorCode::kRecoveryFailed;
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseOutsideTolerance) {
    return ErrorCode::kExecutionFailed;
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseSampleStale) {
    return ErrorCode::kRobotStateStale;
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseSampleInvalid) {
    return ErrorCode::kTfError;
  }
  switch (step.stop_cause) {
    case StopCause::kGoalReached:
      return ErrorCode::kNone;
    case StopCause::kCancelRequested:
      return ErrorCode::kCanceled;
    case StopCause::kGoalTimeout:
      return ErrorCode::kTimeout;
    case StopCause::kPoseSampleStale:
      return ErrorCode::kRobotStateStale;
    case StopCause::kPoseSampleInvalid:
      return ErrorCode::kTfError;
    case StopCause::kBackendSafetyStop:
      return pending_failure ? pending_failure->error_code : ErrorCode::kExecutionFailed;
    case StopCause::kNone:
    case StopCause::kPauseRequested:
      return pending_failure ? pending_failure->error_code : ErrorCode::kInternalError;
  }
  return ErrorCode::kInternalError;
}

std::string terminal_detail(
  const manipulation::servo::ServoToPoseStepResult & step,
  const std::optional<PendingFailure> & pending_failure)
{
  if (step.terminal_reason == TerminalReason::kHaltTimeout) {
    std::string detail{"ServoToPose could not confirm a bounded safe stop"};
    if (pending_failure) {
      detail += " after: " + pending_failure->detail;
    }
    return detail;
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseOutsideTolerance) {
    return "ServoToPose stopped safely, but the final TCP pose is outside tolerance";
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseSampleStale) {
    return "ServoToPose stopped safely, but no fresh final TCP pose was available";
  }
  if (step.terminal_reason == TerminalReason::kFinalPoseSampleInvalid) {
    return "ServoToPose stopped safely, but the final TCP pose sample was invalid";
  }
  if (pending_failure && step.stop_cause == StopCause::kBackendSafetyStop) {
    return pending_failure->detail;
  }
  switch (step.stop_cause) {
    case StopCause::kGoalReached:
      return "ServoToPose target reached; stopped TCP pose and backend pause confirmed";
    case StopCause::kCancelRequested:
      return "ServoToPose canceled after a confirmed safe stop";
    case StopCause::kGoalTimeout:
      return "ServoToPose exceeded its absolute steady-clock deadline";
    case StopCause::kPoseSampleStale:
      return "ServoToPose stopped because robot pose feedback became stale";
    case StopCause::kPoseSampleInvalid:
      return "ServoToPose stopped because pose feedback became invalid";
    case StopCause::kBackendSafetyStop:
      return "MoveIt Servo reported a safety stop";
    case StopCause::kNone:
    case StopCause::kPauseRequested:
      return "ServoToPose terminated without a valid terminal cause";
  }
  return "ServoToPose terminated with an unknown cause";
}

}  // namespace manipulation::moveit::servo_to_pose_detail
