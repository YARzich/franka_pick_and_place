// Responsibility: Starts reserved neutral sequence work and advances its heartbeat pump.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "detail/transfer_sequence_action_active_goal.hpp"
#include "manipulation/workflow/detail/transfer_sequence_task_context_factory.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using ObjectDisposition = manipulation::task::ObjectDisposition;
using TaskErrorCode = manipulation::task::TaskErrorCode;
using TaskOutcome = manipulation::task::TaskOutcome;

TaskOutcome stopped_before_backend(
  const TaskStopCause cause,
  const ObjectDisposition disposition,
  const std::string & action_label)
{
  switch (cause) {
    case TaskStopCause::kClientCanceled:
      return TaskOutcome::make_failure(
        TaskErrorCode::kCanceled, disposition, 0U,
        action_label + " canceled before backend startup");
    case TaskStopCause::kDeadlineExceeded:
      return TaskOutcome::make_failure(
        TaskErrorCode::kTimeout, disposition, 0U,
        action_label + " deadline expired before backend startup");
    case TaskStopCause::kShutdown:
      return TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, disposition, 0U,
        action_label + " server shut down before backend startup");
    case TaskStopCause::kNone:
      return TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, ObjectDisposition::kUnknown, 0U,
        "pre-start sequence stop requested without a cause");
  }
  return TaskOutcome::make_failure(
    TaskErrorCode::kInternalError, ObjectDisposition::kUnknown, 0U,
    "unknown pre-start sequence stop cause");
}

}  // namespace

void TransferSequenceActionLifecycleCore::start_reserved_goal_if_ready()
{
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_reserved_ || !worker_start_pending_ || worker_start_claimed_ ||
      !active_goal_)
    {
      return;
    }
    if (Clock::now() < worker_start_not_before_ &&
      stop_cause(active_goal_) == TaskStopCause::kNone)
    {
      return;
    }
    worker_start_claimed_ = true;
    worker_start_pending_ = false;
    pending_transport_.reset();
    active = active_goal_;
  }

  const auto cause = stop_cause(active);
  if (cause != TaskStopCause::kNone) {
    queue_completion(
      active,
      stopped_before_backend(cause, active->initial_disposition, action_label_));
    return;
  }
  if (!active->session.mark_worker_starting()) {
    queue_completion(
      active,
      TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, active->initial_disposition, 0U,
        action_label_ + " session could not enter worker-starting state"));
    return;
  }

  try {
    const auto start_result = worker_.start(
      [this, active](manipulation::task::ExecutionControl & control) {
        return execute_backend(active, control);
      },
      [this, active](TaskOutcome outcome) {
        queue_completion(active, std::move(outcome));
      });
    if (start_result == TaskStartResult::kStarted) {
      (void)active->session.mark_worker_running();
      return;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "Failed to start %s worker: %s", action_label_.c_str(), error.what());
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "Failed to start %s worker: unknown exception", action_label_.c_str());
  }

  queue_completion(
    active,
    TaskOutcome::make_failure(
      TaskErrorCode::kInternalError, ObjectDisposition::kUnknown, 0U,
      "managed " + action_label_ + " worker could not start"));
}

TaskOutcome TransferSequenceActionLifecycleCore::execute_backend(
  const std::shared_ptr<ActiveGoal> & active,
  manipulation::task::ExecutionControl & control)
{
  if (active->session.pause_requested()) {
    control.request_pause();
  }

  auto context = TransferSequenceTaskContextFactory::create(
    control,
    active->deadline,
    [this, active](TransferSequenceFeedbackSnapshot feedback) {
      accept_feedback(active, std::move(feedback));
    },
    [this, active]() {return stop_cause(active);},
    [this, active]() {
      (void)set_stop_cause_if_none(active, TaskStopCause::kDeadlineExceeded);
    },
    [active](const TransferSequenceTaskContext::Operation & operation) {
      return active->session.run_operation(active->deadline, operation);
    },
    [active](const TransferSequenceTaskContext::Operation & operation) {
      return active->session.run_maintenance_operation(active->deadline, operation);
    });

  if (context.stop_cause() != TaskStopCause::kNone) {
    return context.make_stopped_outcome(active->initial_disposition);
  }
  if (!active->session.mark_backend_invoked()) {
    return TaskOutcome::make_failure(
      TaskErrorCode::kInternalError, active->initial_disposition, 0U,
      action_label_ + " session rejected backend invocation");
  }

  auto execution = active->execute(context);
  const auto outcome = execution.result.outcome;
  {
    const std::lock_guard<std::mutex> lock(active->execution_mutex);
    if (active->backend_execution) {
      return TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, ObjectDisposition::kUnknown, 0U,
        action_label_ + " backend produced duplicate execution evidence");
    }
    active->backend_execution.emplace(std::move(execution));
  }
  return outcome;
}

void TransferSequenceActionLifecycleCore::heartbeat()
{
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
  }
  if (!active) {
    release_completed_reservation();
    return;
  }

  if (!completion_is_queued(active) && Clock::now() >= active->deadline) {
    (void)set_stop_cause_if_none(active, TaskStopCause::kDeadlineExceeded);
  }
  start_reserved_goal_if_ready();

  const auto cause = stop_cause(active);
  if (cause != TaskStopCause::kNone) {
    if (active->session.snapshot().worker_started &&
      !active->cancel_propagated.exchange(true))
    {
      (void)worker_.request_cancel();
    }
    dispatch_backend_stop(active);
  }

  auto pending = active->session.take_completion();
  if (pending) {
    complete_goal(active, std::move(*pending));
    release_completed_reservation();
    return;
  }
  if (active->session.snapshot().phase != TransferSequenceSessionPhase::kDelivered) {
    publish_feedback(active);
  }
  release_completed_reservation();
}

}  // namespace manipulation::workflow::detail
