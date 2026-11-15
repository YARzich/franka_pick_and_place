// Responsibility: Validates neutral feedback and dispatches one external stop request.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <utility>

#include "detail/transfer_sequence_action_active_goal.hpp"

namespace manipulation::workflow::detail
{
namespace
{

float normalized_progress(const float progress)
{
  return std::isfinite(progress) ? std::clamp(progress, 0.0F, 1.0F) : 0.0F;
}

}  // namespace

void TransferSequenceActionLifecycleCore::accept_feedback(
  const std::shared_ptr<ActiveGoal> & active,
  TransferSequenceFeedbackSnapshot feedback)
{
  if (!is_current_goal(active)) {
    RCLCPP_WARN(logger_, "Ignoring stale %s backend feedback", action_label_.c_str());
    return;
  }
  if (feedback.step_count != active->step_count ||
    feedback.step_index >= active->step_count ||
    feedback.step_index > std::numeric_limits<std::uint32_t>::max())
  {
    RCLCPP_WARN(
      logger_, "Ignoring invalid %s backend feedback coordinates",
      action_label_.c_str());
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(active->feedback_mutex);
    if (feedback.step_index < active->feedback.step_index) {
      RCLCPP_WARN(
        logger_, "Ignoring regressive %s step feedback", action_label_.c_str());
      return;
    }
    feedback.overall_progress = std::max(
      active->feedback.overall_progress,
      normalized_progress(feedback.overall_progress));
    active->feedback = std::move(feedback);
  }
  publish_feedback(active);
}

void TransferSequenceActionLifecycleCore::publish_feedback(
  const std::shared_ptr<ActiveGoal> & active)
{
  const std::lock_guard<std::mutex> publish_lock(active->publish_mutex);
  if (!is_current_goal(active)) {
    return;
  }

  try {
    if (active->transport->state() != SequenceGoalTransportState::kExecuting) {
      return;
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger_, "Could not query %s feedback transport state: %s",
      action_label_.c_str(), error.what());
    return;
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Could not query %s feedback transport state: unknown exception",
      action_label_.c_str());
    return;
  }

  auto feedback_state = [&active]() {
      const std::lock_guard<std::mutex> lock(active->feedback_mutex);
      return active->feedback;
    }();
  const auto worker = worker_.snapshot();
  const auto session = active->session.snapshot();
  if (session.worker_started) {
    // An active inner task can acknowledge pause while the outer worker is
    // blocked inside an injected transport or action port.
    feedback_state.current_state.paused = feedback_state.current_state.paused ||
      worker.execution_control.paused_at_checkpoint;
    feedback_state.current_state.pause_pending =
      !feedback_state.current_state.paused &&
      (feedback_state.current_state.pause_pending || session.pause_requested ||
      worker.execution_control.pause_requested);
  } else {
    feedback_state.current_state.paused = false;
    feedback_state.current_state.pause_pending = session.pause_requested;
  }
  // The wire actions have no separate outer elapsed field. Preserve backend
  // phase and step-local counters while exposing the one steady goal deadline.
  feedback_state.current_state.elapsed = Clock::now() - active->started_at;
  feedback_state.overall_progress = normalized_progress(
    feedback_state.overall_progress);

  try {
    active->transport->publish_feedback(feedback_state);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger_, "Could not publish %s feedback: %s", action_label_.c_str(), error.what());
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Could not publish %s feedback: unknown exception",
      action_label_.c_str());
  }
}

void TransferSequenceActionLifecycleCore::publish_terminal_feedback(
  const std::shared_ptr<ActiveGoal> & active,
  const manipulation::task::TaskOutcome & outcome)
{
  TransferSequenceFeedbackSnapshot terminal;
  {
    const std::lock_guard<std::mutex> lock(active->feedback_mutex);
    terminal = active->feedback;
  }
  terminal.current_state.phase = outcome.succeeded() ?
    manipulation::task::TaskPhase::kCompleted :
    (outcome.error_code() == manipulation::task::TaskErrorCode::kCanceled ?
    manipulation::task::TaskPhase::kCanceled : manipulation::task::TaskPhase::kFailed);
  terminal.overall_progress = outcome.succeeded() ? 1.0F : terminal.overall_progress;
  terminal.current_state.progress = outcome.succeeded() ?
    1.0F : terminal.current_state.progress;
  if (outcome.succeeded() && terminal.step_count > 0U) {
    terminal.step_index = terminal.step_count - 1U;
  }
  terminal.current_state.detail = outcome.detail();
  accept_feedback(active, std::move(terminal));
}

TaskStopCause TransferSequenceActionLifecycleCore::stop_cause(
  const std::shared_ptr<ActiveGoal> & active) const
{
  return active->session.stop_cause();
}

bool TransferSequenceActionLifecycleCore::set_stop_cause_if_none(
  const std::shared_ptr<ActiveGoal> & active,
  const TaskStopCause cause)
{
  return active->session.request_stop(cause);
}

void TransferSequenceActionLifecycleCore::dispatch_backend_stop(
  const std::shared_ptr<ActiveGoal> & active)
{
  try {
    // dispatch_stop releases the session mutex before entering backend code.
    (void)active->session.dispatch_stop(request_stop_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "%s request_stop callback failed: %s",
      action_label_.c_str(), error.what());
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "%s request_stop callback failed: unknown exception",
      action_label_.c_str());
  }
}

}  // namespace manipulation::workflow::detail
