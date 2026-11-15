// Responsibility: Implements deterministic ServoToPose tracking, pause, and bounded halt rules.

#include "manipulation/servo/servo_to_pose_state_machine.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace manipulation::servo
{
namespace
{

bool finite_nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

int stop_priority(const ServoToPoseStopCause cause) noexcept
{
  switch (cause) {
    case ServoToPoseStopCause::kNone:
      return 0;
    case ServoToPoseStopCause::kPauseRequested:
      return 1;
    case ServoToPoseStopCause::kGoalReached:
      return 2;
    case ServoToPoseStopCause::kPoseSampleStale:
      return 3;
    case ServoToPoseStopCause::kPoseSampleInvalid:
      return 4;
    case ServoToPoseStopCause::kBackendSafetyStop:
      return 5;
    case ServoToPoseStopCause::kGoalTimeout:
      return 6;
    case ServoToPoseStopCause::kCancelRequested:
      return 7;
  }
  return 0;
}

}  // namespace

ServoToPoseStateMachine::ServoToPoseStateMachine(ServoToPoseStateMachineConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.position_tolerance_m) ||
    config_.position_tolerance_m <= 0.0)
  {
    throw std::invalid_argument("ServoToPose position tolerance must be finite and positive");
  }
  if (!std::isfinite(config_.orientation_tolerance_rad) ||
    config_.orientation_tolerance_rad <= 0.0)
  {
    throw std::invalid_argument("ServoToPose orientation tolerance must be finite and positive");
  }
  if (!finite_nonnegative(config_.stopped_velocity_tolerance)) {
    throw std::invalid_argument(
            "ServoToPose stopped-velocity tolerance must be finite and non-negative");
  }
  if (config_.required_target_samples == 0U || config_.required_stopped_samples == 0U) {
    throw std::invalid_argument("ServoToPose evidence sample counts must be positive");
  }
  if (config_.goal_timeout <= Duration::zero() || config_.halt_timeout <= Duration::zero()) {
    throw std::invalid_argument("ServoToPose goal and halt timeouts must be positive");
  }
}

ServoToPoseStepResult ServoToPoseStateMachine::step(const ServoToPoseStepInput & input)
{
  if (input.elapsed < Duration::zero()) {
    throw std::invalid_argument("ServoToPose elapsed time must be non-negative");
  }
  if (elapsed_initialized_ && input.elapsed < last_elapsed_) {
    throw std::invalid_argument("ServoToPose elapsed time must be monotonic");
  }
  elapsed_initialized_ = true;
  last_elapsed_ = input.elapsed;

  if (state_ == ServoToPoseState::kTerminal) {
    return snapshot();
  }

  const auto state_before_priority_stop = state_;
  if (handle_priority_stop(input)) {
    if (state_ == ServoToPoseState::kTerminal) {
      return snapshot();
    }
    if (state_before_priority_stop != ServoToPoseState::kHalting) {
      return snapshot();
    }
  }

  switch (state_) {
    case ServoToPoseState::kAwaitingResume:
      return step_awaiting_resume(input);
    case ServoToPoseState::kTracking:
      return step_tracking(input);
    case ServoToPoseState::kHalting:
      return step_halting(input);
    case ServoToPoseState::kPaused:
      return step_paused(input);
    case ServoToPoseState::kTerminal:
      return snapshot();
  }
  throw std::logic_error("ServoToPose entered an unknown state");
}

ServoToPoseStepResult ServoToPoseStateMachine::snapshot() const noexcept
{
  return make_result(ServoToPoseCommand::kNone);
}

const ServoToPoseStateMachineConfig & ServoToPoseStateMachine::config() const noexcept
{
  return config_;
}

ServoToPoseStepResult ServoToPoseStateMachine::make_result(
  const ServoToPoseCommand command) const noexcept
{
  return ServoToPoseStepResult{
    state_, halt_phase_, command, stop_cause_, terminal_reason_, stop_confirmed_,
    consecutive_target_samples_, consecutive_stopped_samples_,
    consecutive_final_pose_samples_};
}

ServoToPoseStepResult ServoToPoseStateMachine::step_awaiting_resume(
  const ServoToPoseStepInput & input)
{
  if (input.pause_requested) {
    return begin_halt(ServoToPoseStopCause::kPauseRequested, input.elapsed);
  }

  if (!resume_command_issued_) {
    resume_command_issued_ = true;
    return make_result(ServoToPoseCommand::kRequestResume);
  }
  if (input.backend_state != ServoBackendState::kRunning) {
    return make_result(ServoToPoseCommand::kRequestResume);
  }

  enter_tracking();
  return step_tracking(input);
}

ServoToPoseStepResult ServoToPoseStateMachine::step_tracking(
  const ServoToPoseStepInput & input)
{
  if (input.pause_requested) {
    return begin_halt(ServoToPoseStopCause::kPauseRequested, input.elapsed);
  }
  if (input.backend_state != ServoBackendState::kRunning) {
    return begin_halt(ServoToPoseStopCause::kBackendSafetyStop, input.elapsed);
  }

  const auto & sample = input.pose_sample;
  if (!sample.available || !sample.fresh) {
    return begin_halt(ServoToPoseStopCause::kPoseSampleStale, input.elapsed);
  }
  if (!sample.valid || sample.sequence == 0U ||
    !finite_nonnegative(sample.position_error_m) ||
    !finite_nonnegative(sample.orientation_error_rad) ||
    (pose_sequence_initialized_ && sample.sequence < last_pose_sequence_))
  {
    return begin_halt(ServoToPoseStopCause::kPoseSampleInvalid, input.elapsed);
  }

  if (pose_sample_is_new(sample)) {
    const bool target_reached =
      sample.position_error_m <= config_.position_tolerance_m &&
      sample.orientation_error_rad <= config_.orientation_tolerance_rad;
    if (target_reached) {
      ++consecutive_target_samples_;
    } else {
      consecutive_target_samples_ = 0U;
    }
  }

  if (consecutive_target_samples_ >= config_.required_target_samples) {
    return begin_halt(ServoToPoseStopCause::kGoalReached, input.elapsed);
  }
  return make_result(ServoToPoseCommand::kPublishTarget);
}

ServoToPoseStepResult ServoToPoseStateMachine::step_halting(
  const ServoToPoseStepInput & input)
{
  if (input.elapsed - halt_started_at_ >= config_.halt_timeout) {
    if (halt_phase_ == ServoToPoseHaltPhase::kVerifyingFinalPose && stop_confirmed_) {
      halt_phase_ = ServoToPoseHaltPhase::kNone;
      const auto reason = stop_cause_ == ServoToPoseStopCause::kGoalReached ?
        ServoToPoseTerminalReason::kFinalPoseSampleStale :
        terminal_reason_for_stop_cause();
      enter_terminal(reason, true);
    } else {
      enter_terminal(ServoToPoseTerminalReason::kHaltTimeout, false);
    }
    return snapshot();
  }

  if (halt_phase_ == ServoToPoseHaltPhase::kVerifyingFinalPose) {
    return step_verifying_final_pose(input);
  }

  if (halt_phase_ == ServoToPoseHaltPhase::kAwaitingBackendPause) {
    if (input.backend_state != ServoBackendState::kPaused) {
      return make_result(ServoToPoseCommand::kRequestPause);
    }
    return complete_confirmed_halt(input);
  }

  const auto & sample = input.velocity_sample;
  if (!sample.available) {
    consecutive_stopped_samples_ = 0U;
    return make_result(ServoToPoseCommand::kNone);
  }
  if (!sample.fresh || !sample.valid || sample.sequence == 0U ||
    !finite_nonnegative(sample.max_abs_joint_velocity) ||
    (velocity_sequence_initialized_ && sample.sequence < last_velocity_sequence_))
  {
    if (sample.sequence != 0U &&
      (!velocity_sequence_initialized_ || sample.sequence > last_velocity_sequence_))
    {
      last_velocity_sequence_ = sample.sequence;
      velocity_sequence_initialized_ = true;
    }
    consecutive_stopped_samples_ = 0U;
    return make_result(ServoToPoseCommand::kNone);
  }

  if (velocity_sample_is_new(sample)) {
    if (sample.max_abs_joint_velocity <= config_.stopped_velocity_tolerance) {
      ++consecutive_stopped_samples_;
    } else {
      consecutive_stopped_samples_ = 0U;
    }
  }

  if (consecutive_stopped_samples_ < config_.required_stopped_samples) {
    return make_result(ServoToPoseCommand::kNone);
  }

  if (input.backend_state == ServoBackendState::kPaused) {
    return complete_confirmed_halt(input);
  }

  halt_phase_ = ServoToPoseHaltPhase::kAwaitingBackendPause;
  return make_result(ServoToPoseCommand::kRequestPause);
}

ServoToPoseStepResult ServoToPoseStateMachine::step_verifying_final_pose(
  const ServoToPoseStepInput & input)
{
  if (input.backend_state != ServoBackendState::kPaused) {
    stop_confirmed_ = false;
    escalate_stop_cause(ServoToPoseStopCause::kBackendSafetyStop);
    halt_phase_ = ServoToPoseHaltPhase::kWaitingForLowVelocity;
    reset_stop_evidence();
    reset_final_pose_evidence();
    return make_result(ServoToPoseCommand::kNone);
  }

  if (stop_cause_ != ServoToPoseStopCause::kGoalReached) {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    enter_terminal(terminal_reason_for_stop_cause(), true);
    return snapshot();
  }

  const auto & sample = input.pose_sample;
  if (!sample.available || !sample.fresh) {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    enter_terminal(ServoToPoseTerminalReason::kFinalPoseSampleStale, true);
    return snapshot();
  }
  if (!sample.valid || sample.sequence == 0U ||
    !finite_nonnegative(sample.position_error_m) ||
    !finite_nonnegative(sample.orientation_error_rad) ||
    (final_pose_sequence_initialized_ && sample.sequence < last_final_pose_sequence_))
  {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    enter_terminal(ServoToPoseTerminalReason::kFinalPoseSampleInvalid, true);
    return snapshot();
  }

  if (!final_pose_sample_is_new(sample)) {
    return make_result(ServoToPoseCommand::kNone);
  }

  const bool target_reached =
    sample.position_error_m <= config_.position_tolerance_m &&
    sample.orientation_error_rad <= config_.orientation_tolerance_rad;
  if (!target_reached) {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    enter_terminal(ServoToPoseTerminalReason::kFinalPoseOutsideTolerance, true);
    return snapshot();
  }

  ++consecutive_final_pose_samples_;
  if (consecutive_final_pose_samples_ >= config_.required_target_samples) {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    enter_terminal(ServoToPoseTerminalReason::kSucceeded, true);
    return snapshot();
  }
  return make_result(ServoToPoseCommand::kNone);
}

ServoToPoseStepResult ServoToPoseStateMachine::step_paused(
  const ServoToPoseStepInput & input)
{
  if (input.pause_requested && input.backend_state == ServoBackendState::kPaused) {
    return snapshot();
  }
  if (input.pause_requested) {
    return begin_halt(ServoToPoseStopCause::kPauseRequested, input.elapsed);
  }

  state_ = ServoToPoseState::kAwaitingResume;
  halt_phase_ = ServoToPoseHaltPhase::kNone;
  stop_cause_ = ServoToPoseStopCause::kNone;
  terminal_reason_ = ServoToPoseTerminalReason::kNone;
  resume_command_issued_ = true;
  stop_confirmed_ = false;
  reset_target_evidence();
  reset_stop_evidence();
  return make_result(ServoToPoseCommand::kRequestResume);
}

bool ServoToPoseStateMachine::handle_priority_stop(const ServoToPoseStepInput & input)
{
  ServoToPoseStopCause requested_cause{ServoToPoseStopCause::kNone};
  if (input.cancel_requested) {
    requested_cause = ServoToPoseStopCause::kCancelRequested;
  } else if (input.elapsed >= config_.goal_timeout) {
    requested_cause = ServoToPoseStopCause::kGoalTimeout;
  } else if (input.safety_status == ServoSafetyStatus::kStopRequired) {
    requested_cause = ServoToPoseStopCause::kBackendSafetyStop;
  }

  if (requested_cause == ServoToPoseStopCause::kNone) {
    return false;
  }
  if (state_ == ServoToPoseState::kPaused && stop_confirmed_) {
    stop_cause_ = requested_cause;
    enter_terminal(terminal_reason_for_stop_cause(), true);
    return true;
  }
  if (state_ == ServoToPoseState::kHalting) {
    escalate_stop_cause(requested_cause);
    return true;
  }

  (void)begin_halt(requested_cause, input.elapsed);
  return true;
}

ServoToPoseStepResult ServoToPoseStateMachine::begin_halt(
  const ServoToPoseStopCause cause,
  const Duration elapsed)
{
  state_ = ServoToPoseState::kHalting;
  halt_phase_ = ServoToPoseHaltPhase::kWaitingForLowVelocity;
  stop_cause_ = cause;
  terminal_reason_ = ServoToPoseTerminalReason::kNone;
  stop_confirmed_ = false;
  halt_started_at_ = elapsed;
  resume_command_issued_ = false;
  reset_stop_evidence();
  reset_final_pose_evidence();
  return make_result(ServoToPoseCommand::kNone);
}

ServoToPoseStepResult ServoToPoseStateMachine::complete_confirmed_halt(
  const ServoToPoseStepInput & input)
{
  stop_confirmed_ = true;
  if (stop_cause_ == ServoToPoseStopCause::kPauseRequested) {
    halt_phase_ = ServoToPoseHaltPhase::kNone;
    state_ = ServoToPoseState::kPaused;
    return snapshot();
  }
  if (stop_cause_ == ServoToPoseStopCause::kGoalReached) {
    enter_final_pose_verification(input.pose_sample);
    return snapshot();
  }

  halt_phase_ = ServoToPoseHaltPhase::kNone;
  enter_terminal(terminal_reason_for_stop_cause(), true);
  return snapshot();
}

void ServoToPoseStateMachine::escalate_stop_cause(const ServoToPoseStopCause cause) noexcept
{
  if (stop_priority(cause) > stop_priority(stop_cause_)) {
    stop_cause_ = cause;
  }
}

void ServoToPoseStateMachine::enter_tracking() noexcept
{
  state_ = ServoToPoseState::kTracking;
  halt_phase_ = ServoToPoseHaltPhase::kNone;
  stop_cause_ = ServoToPoseStopCause::kNone;
  terminal_reason_ = ServoToPoseTerminalReason::kNone;
  stop_confirmed_ = false;
  resume_command_issued_ = false;
  reset_target_evidence();
  reset_stop_evidence();
  reset_final_pose_evidence();
}

void ServoToPoseStateMachine::enter_final_pose_verification(
  const ServoPoseSample & baseline) noexcept
{
  halt_phase_ = ServoToPoseHaltPhase::kVerifyingFinalPose;
  reset_final_pose_evidence();
  if (baseline.sequence != 0U) {
    final_pose_sequence_initialized_ = true;
    last_final_pose_sequence_ = baseline.sequence;
  }
}

void ServoToPoseStateMachine::enter_terminal(
  const ServoToPoseTerminalReason reason,
  const bool stop_confirmed) noexcept
{
  state_ = ServoToPoseState::kTerminal;
  terminal_reason_ = reason;
  stop_confirmed_ = stop_confirmed;
  resume_command_issued_ = false;
}

void ServoToPoseStateMachine::reset_target_evidence() noexcept
{
  consecutive_target_samples_ = 0U;
  pose_sequence_initialized_ = false;
  last_pose_sequence_ = 0U;
}

void ServoToPoseStateMachine::reset_stop_evidence() noexcept
{
  consecutive_stopped_samples_ = 0U;
  velocity_sequence_initialized_ = false;
  last_velocity_sequence_ = 0U;
}

void ServoToPoseStateMachine::reset_final_pose_evidence() noexcept
{
  consecutive_final_pose_samples_ = 0U;
  final_pose_sequence_initialized_ = false;
  last_final_pose_sequence_ = 0U;
}

ServoToPoseTerminalReason ServoToPoseStateMachine::terminal_reason_for_stop_cause()
const noexcept
{
  switch (stop_cause_) {
    case ServoToPoseStopCause::kGoalReached:
      return ServoToPoseTerminalReason::kSucceeded;
    case ServoToPoseStopCause::kCancelRequested:
      return ServoToPoseTerminalReason::kCanceled;
    case ServoToPoseStopCause::kGoalTimeout:
      return ServoToPoseTerminalReason::kGoalTimeout;
    case ServoToPoseStopCause::kPoseSampleStale:
      return ServoToPoseTerminalReason::kPoseSampleStale;
    case ServoToPoseStopCause::kPoseSampleInvalid:
      return ServoToPoseTerminalReason::kPoseSampleInvalid;
    case ServoToPoseStopCause::kBackendSafetyStop:
      return ServoToPoseTerminalReason::kBackendSafetyStop;
    case ServoToPoseStopCause::kNone:
    case ServoToPoseStopCause::kPauseRequested:
      return ServoToPoseTerminalReason::kNone;
  }
  return ServoToPoseTerminalReason::kNone;
}

bool ServoToPoseStateMachine::pose_sample_is_new(const ServoPoseSample & sample) noexcept
{
  if (!pose_sequence_initialized_) {
    pose_sequence_initialized_ = true;
    last_pose_sequence_ = sample.sequence;
    return true;
  }
  if (sample.sequence == last_pose_sequence_) {
    return false;
  }
  last_pose_sequence_ = sample.sequence;
  return true;
}

bool ServoToPoseStateMachine::velocity_sample_is_new(
  const ServoVelocitySample & sample) noexcept
{
  if (!velocity_sequence_initialized_) {
    velocity_sequence_initialized_ = true;
    last_velocity_sequence_ = sample.sequence;
    return true;
  }
  if (sample.sequence == last_velocity_sequence_) {
    return false;
  }
  last_velocity_sequence_ = sample.sequence;
  return true;
}

bool ServoToPoseStateMachine::final_pose_sample_is_new(
  const ServoPoseSample & sample) noexcept
{
  if (!final_pose_sequence_initialized_) {
    final_pose_sequence_initialized_ = true;
    last_final_pose_sequence_ = sample.sequence;
    return true;
  }
  if (sample.sequence == last_final_pose_sequence_) {
    return false;
  }
  last_final_pose_sequence_ = sample.sequence;
  return true;
}

}  // namespace manipulation::servo
