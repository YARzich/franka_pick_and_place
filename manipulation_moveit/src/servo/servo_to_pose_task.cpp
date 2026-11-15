// Responsibility: Executes one fixed reactive pose target with bounded fail-closed stopping.

#include "manipulation_moveit/servo/servo_to_pose_task.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "manipulation/servo/servo_to_pose_state_machine.hpp"
#include "servo_to_pose_task_policy.hpp"

namespace manipulation::moveit
{
namespace
{

using Clock = ServoToPoseTask::Clock;
using ErrorCode = manipulation::task::TaskErrorCode;
using ServoBackendState = manipulation::servo::ServoBackendState;
using ServoCommand = manipulation::servo::ServoToPoseCommand;
using ServoHaltPhase = manipulation::servo::ServoToPoseHaltPhase;
using ServoSafetyStatus = manipulation::servo::ServoSafetyStatus;
using ServoState = manipulation::servo::ServoToPoseState;
using servo_to_pose_detail::PendingFailure;
using servo_to_pose_detail::bounded_deadline;
using servo_to_pose_detail::emit_feedback;
using servo_to_pose_detail::make_feedback;
using servo_to_pose_detail::nonnegative_elapsed;
using servo_to_pose_detail::observation_failure;
using servo_to_pose_detail::setup_error_code;
using servo_to_pose_detail::startup_feedback_ready;
using servo_to_pose_detail::terminal_detail;
using servo_to_pose_detail::terminal_error_code;

bool expected_paused_status_silence(
  const manipulation::servo::ServoToPoseStepResult & machine,
  const MoveItServoRosObservation & observation) noexcept
{
  const bool pause_boundary =
    (machine.state == ServoState::kPaused && machine.stop_confirmed) ||
    (machine.state == ServoState::kHalting &&
    machine.halt_phase == ServoHaltPhase::kAwaitingBackendPause);
  return pause_boundary && observation.backend_state == ServoBackendState::kPaused &&
         observation.status.available && observation.status.valid &&
         !observation.status.fresh && !observation.ros_time_moved_backwards;
}

}  // namespace

ServoToPoseTaskResult ServoToPoseTask::run(
  const geometry_msgs::msg::PoseStamped & target,
  const Clock::time_point started_at,
  const Clock::time_point absolute_deadline,
  ServoToPoseTaskControl & control,
  FeedbackCallback feedback_callback)
{
  try {
    return run_impl(
      target, started_at, absolute_deadline, control, feedback_callback);
  } catch (const std::exception & error) {
    return halt_after_unexpected_failure(
      started_at, absolute_deadline,
      std::string{"ServoToPose task exception: "} + error.what());
  } catch (...) {
    return halt_after_unexpected_failure(
      started_at, absolute_deadline,
      "ServoToPose task raised an unknown exception");
  }
}

ServoToPoseTaskResult ServoToPoseTask::run_impl(
  const geometry_msgs::msg::PoseStamped & target,
  const Clock::time_point started_at,
  const Clock::time_point absolute_deadline,
  ServoToPoseTaskControl & control,
  const FeedbackCallback & feedback_callback)
{
  if (!feedback_callback) {
    throw std::invalid_argument("ServoToPose feedback callback must be callable");
  }
  if (absolute_deadline <= started_at) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kTimeout,
      "ServoToPose absolute deadline was already expired");
  }
  const auto complete_budget = absolute_deadline - started_at;
  if (!servo_to_pose_timeout_has_operational_budget(complete_budget, config_)) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kTimeout,
      "ServoToPose deadline has no usable tracking budget before bounded halt");
  }
  const auto operational_deadline = absolute_deadline - config_.state_machine.halt_timeout;

  ServoToPoseTaskFeedback validating;
  validating.state.phase = manipulation::task::TaskPhase::kValidating;
  validating.state.step_count = 1U;
  validating.state.detail = "pausing and configuring MoveIt Servo before target preparation";
  emit_feedback(feedback_callback, validating);

  auto status = port_.pause(bounded_deadline(operational_deadline, config_.service_call_timeout));
  if (!status.succeeded()) {
    const auto canceled = control.snapshot().cancel_requested;
    return halt_after_failure(
      started_at, absolute_deadline,
      canceled ? ErrorCode::kCanceled :
      (Clock::now() >= operational_deadline ? ErrorCode::kTimeout :
      setup_error_code(status.error(), false)),
      "could not establish initial Servo pause: " + status.detail());
  }
  if (control.snapshot().cancel_requested) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kCanceled,
      "ServoToPose canceled before command-mode configuration");
  }
  if (Clock::now() >= operational_deadline) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kTimeout,
      "ServoToPose operational deadline expired after initial pause");
  }
  status = port_.configure_pose_mode(
    bounded_deadline(operational_deadline, config_.service_call_timeout));
  if (!status.succeeded()) {
    const auto canceled = control.snapshot().cancel_requested;
    return halt_after_failure(
      started_at, absolute_deadline,
      canceled ? ErrorCode::kCanceled :
      (Clock::now() >= operational_deadline ? ErrorCode::kTimeout :
      setup_error_code(status.error(), false)),
      "could not configure Servo POSE mode: " + status.detail());
  }
  if (control.snapshot().cancel_requested) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kCanceled,
      "ServoToPose canceled before exact-stamp target preparation");
  }
  if (Clock::now() >= operational_deadline) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kTimeout,
      "ServoToPose operational deadline expired before exact-stamp target preparation");
  }
  const auto prepared = port_.prepare_target(
    target, bounded_deadline(operational_deadline, config_.target_prepare_timeout));
  if (!prepared.status.succeeded() || !prepared.target) {
    const auto canceled = control.snapshot().cancel_requested;
    return halt_after_failure(
      started_at, absolute_deadline,
      canceled ? ErrorCode::kCanceled :
      (Clock::now() >= operational_deadline ? ErrorCode::kTimeout :
      setup_error_code(prepared.status.error(), true)),
      "could not prepare exact-stamp Servo target: " + prepared.status.detail());
  }
  if (Clock::now() >= operational_deadline) {
    return halt_after_failure(
      started_at, absolute_deadline, ErrorCode::kTimeout,
      "ServoToPose operational deadline expired during exact-stamp target preparation");
  }

  auto machine_config = config_.state_machine;
  machine_config.goal_timeout = operational_deadline - started_at;
  manipulation::servo::ServoToPoseStateMachine machine{machine_config};

  MoveItServoObservationResult observation{
    MoveItServoRosResult::success("backend starts paused"), {}};
  observation.observation.backend_state = manipulation::servo::ServoBackendState::kPaused;
  observation.observation.safety_status = manipulation::servo::ServoSafetyStatus::kNominal;
  std::optional<PendingFailure> pending_failure;
  std::optional<Clock::time_point> resumed_at;
  auto next_cycle = Clock::now();

  while (true) {
    const auto now = Clock::now();
    const auto elapsed = now > started_at ? now - started_at : Clock::duration::zero();
    if (now >= absolute_deadline) {
      return halt_after_failure(
        started_at, absolute_deadline, ErrorCode::kTimeout,
        "ServoToPose absolute deadline expired before safe-stop confirmation");
    }
    if (now >= operational_deadline && !pending_failure) {
      pending_failure = PendingFailure{
        ErrorCode::kTimeout,
        "ServoToPose operational deadline reached; reserved halt budget is active"};
    }

    if (observation.observation.ros_time_moved_backwards && !pending_failure) {
      pending_failure = PendingFailure{
        ErrorCode::kRobotStateStale,
        "ROS time moved backwards during ServoToPose execution"};
    }

    const auto control_snapshot = control.snapshot();
    const auto machine_snapshot = machine.snapshot();
    if (machine_snapshot.state == ServoState::kAwaitingResume && resumed_at &&
      !control_snapshot.cancel_requested && !control_snapshot.pause_requested &&
      !pending_failure && !startup_feedback_ready(observation))
    {
      if (now - *resumed_at >= config_.feedback_startup_timeout) {
        pending_failure = PendingFailure{
          ErrorCode::kRobotStateStale,
          "MoveIt Servo did not provide coherent fresh status, TF, and joint feedback "
          "within the startup deadline"};
      } else {
        auto waiting = validating;
        waiting.state.elapsed = elapsed;
        waiting.state.detail =
          "waiting for coherent fresh Servo status, TF, and joint feedback";
        emit_feedback(feedback_callback, waiting);

        next_cycle = std::max(next_cycle + config_.control_period, Clock::now());
        const auto current_control = control.snapshot();
        control.wait_until(std::min(next_cycle, operational_deadline), current_control.revision);
        observation = port_.latest_observation(
          bounded_deadline(operational_deadline, config_.observation_timeout));
        if (!observation.status.succeeded() && !pending_failure) {
          pending_failure = observation_failure(observation.status);
        }
        continue;
      }
    }

    manipulation::servo::ServoToPoseStepInput input;
    input.elapsed = elapsed;
    input.cancel_requested = control_snapshot.cancel_requested;
    input.pause_requested = control_snapshot.pause_requested;
    input.backend_state = observation.observation.backend_state;
    const auto backend_safety = expected_paused_status_silence(
      machine_snapshot, observation.observation) ?
      ServoSafetyStatus::kNominal : observation.observation.safety_status;
    input.safety_status = pending_failure ? ServoSafetyStatus::kStopRequired : backend_safety;
    input.pose_sample = observation.observation.pose_sample;
    input.velocity_sample = observation.observation.velocity_sample;

    const auto step = machine.step(input);
    emit_feedback(
      feedback_callback, make_feedback(step, observation, control_snapshot, elapsed));
    if (step.terminal()) {
      return {
        terminal_error_code(step, pending_failure),
        step.stop_confirmed,
        elapsed,
        terminal_detail(step, pending_failure)};
    }

    switch (step.command) {
      case ServoCommand::kNone:
        break;
      case ServoCommand::kRequestResume: {
          const auto resumed = port_.unpause(
            bounded_deadline(operational_deadline, config_.service_call_timeout));
          if (!resumed.succeeded()) {
            pending_failure = PendingFailure{
              setup_error_code(resumed.error(), false),
              "could not resume MoveIt Servo: " + resumed.detail()};
          } else {
            observation.observation.backend_state =
              manipulation::servo::ServoBackendState::kRunning;
            resumed_at = Clock::now();
          }
          break;
        }
      case ServoCommand::kPublishTarget: {
          const auto published = port_.publish_target();
          if (!published.status.succeeded()) {
            pending_failure = PendingFailure{
              setup_error_code(published.status.error(), false),
              "Servo target publication failed: " + published.status.detail()};
          }
          break;
        }
      case ServoCommand::kRequestPause: {
          const auto paused = port_.pause(
            bounded_deadline(absolute_deadline, config_.service_call_timeout));
          if (!paused.succeeded()) {
            pending_failure = PendingFailure{
              setup_error_code(paused.error(), false),
              "MoveIt Servo pause confirmation failed: " + paused.detail()};
          } else {
            observation.observation.backend_state =
              manipulation::servo::ServoBackendState::kPaused;
          }
          break;
        }
    }

    next_cycle = std::max(next_cycle + config_.control_period, Clock::now());
    const auto current_control = control.snapshot();
    const auto cycle_deadline = step.state == ServoState::kHalting ?
      absolute_deadline : operational_deadline;
    control.wait_until(std::min(next_cycle, cycle_deadline), current_control.revision);

    observation = port_.latest_observation(
      bounded_deadline(cycle_deadline, config_.observation_timeout));
    if (!observation.status.succeeded() && !pending_failure) {
      pending_failure = observation_failure(observation.status);
    }
  }
}

ServoToPoseTaskResult ServoToPoseTask::halt_after_unexpected_failure(
  const Clock::time_point started_at,
  const Clock::time_point absolute_deadline,
  std::string detail) noexcept
{
  return halt_after_failure(
    started_at, absolute_deadline, ErrorCode::kInternalError, std::move(detail));
}

ServoToPoseTaskResult ServoToPoseTask::halt_after_failure(
  const Clock::time_point started_at,
  const Clock::time_point absolute_deadline,
  const ErrorCode error_code,
  std::string detail) noexcept
{
  try {
    auto machine_config = config_.state_machine;
    machine_config.goal_timeout = Clock::duration::max();
    manipulation::servo::ServoToPoseStateMachine machine{machine_config};
    const auto recovery_started_at = Clock::now();

    while (Clock::now() < absolute_deadline) {
      auto observation = port_.latest_observation(
        bounded_deadline(absolute_deadline, config_.observation_timeout));
      manipulation::servo::ServoToPoseStepInput input;
      input.elapsed = Clock::now() - recovery_started_at;
      input.backend_state = observation.observation.backend_state;
      input.safety_status = manipulation::servo::ServoSafetyStatus::kStopRequired;
      input.pose_sample = observation.observation.pose_sample;
      input.velocity_sample = observation.observation.velocity_sample;
      const auto step = machine.step(input);
      if (step.terminal()) {
        if (step.stop_confirmed) {
          return {
            error_code,
            true,
            nonnegative_elapsed(started_at),
            std::move(detail)};
        }
        return {
          ErrorCode::kRecoveryFailed,
          false,
          nonnegative_elapsed(started_at),
          detail + "; bounded safe stop was not confirmed"};
      }
      if (step.command == ServoCommand::kRequestPause) {
        const auto paused = port_.pause(
          bounded_deadline(absolute_deadline, config_.service_call_timeout));
        if (!paused.succeeded()) {
          detail += "; safe-stop pause failed: " + paused.detail();
        }
      }
      std::this_thread::sleep_for(config_.control_period);
    }
  } catch (...) {
    // The last-resort path still returns a typed unconfirmed recovery failure.
  }
  return {
    ErrorCode::kRecoveryFailed,
    false,
    nonnegative_elapsed(started_at),
    detail + "; absolute deadline expired during safe stop"};
}

}  // namespace manipulation::moveit
