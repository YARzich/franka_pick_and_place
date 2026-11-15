// Responsibility: Normalizes evidence and retries serialized neutral terminal delivery.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <utility>

#include "detail/transfer_sequence_execution_policy.hpp"
#include "detail/transfer_sequence_action_active_goal.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using TaskErrorCode = manipulation::task::TaskErrorCode;
using TaskOutcome = manipulation::task::TaskOutcome;

TransferSequenceExecution outcome_without_steps(TaskOutcome outcome)
{
  return {TransferSequenceResult{std::move(outcome), 0U, std::nullopt, {}}, {}};
}

SequenceTerminalKind terminal_kind(const TaskOutcome & outcome)
{
  if (outcome.succeeded()) {
    return SequenceTerminalKind::kSucceeded;
  }
  return outcome.error_code() == TaskErrorCode::kCanceled ?
         SequenceTerminalKind::kCanceled : SequenceTerminalKind::kAborted;
}

}  // namespace

void TransferSequenceActionLifecycleCore::queue_completion(
  const std::shared_ptr<ActiveGoal> & active,
  TaskOutcome outcome)
{
  if (!is_current_goal(active)) {
    RCLCPP_WARN(logger_, "Ignoring stale %s completion", action_label_.c_str());
    return;
  }
  if (Clock::now() >= active->deadline) {
    (void)set_stop_cause_if_none(active, TaskStopCause::kDeadlineExceeded);
  }
  if (!active->session.queue_completion(std::move(outcome))) {
    RCLCPP_ERROR(
      logger_, "%s backend attempted duplicate completion", action_label_.c_str());
  }
}

bool TransferSequenceActionLifecycleCore::completion_is_queued(
  const std::shared_ptr<ActiveGoal> & active) const
{
  return active->session.snapshot().completion_ready;
}

void TransferSequenceActionLifecycleCore::complete_goal(
  const std::shared_ptr<ActiveGoal> & active,
  TaskOutcome outcome)
{
  if (!is_current_goal(active)) {
    RCLCPP_WARN(logger_, "Ignoring stale %s terminal delivery", action_label_.c_str());
    return;
  }

  TransferSequenceExecution execution = [&]() {
      const std::lock_guard<std::mutex> lock(active->execution_mutex);
      if (!active->backend_execution) {
        return outcome_without_steps(std::move(outcome));
      }
      auto result = std::move(*active->backend_execution);
      active->backend_execution.reset();
      return result;
    }();

  auto policy_result = apply_transfer_sequence_execution_policy(
    std::move(execution), active->step_count, stop_cause(active), action_label_);
  if (policy_result.contract_error) {
    RCLCPP_ERROR(logger_, "%s", policy_result.contract_error->c_str());
  }
  execution = std::move(policy_result.execution);

  const auto restore_for_retry = [&active](TransferSequenceExecution retry_execution) {
      const auto retry_outcome = retry_execution.result.outcome;
      {
        const std::lock_guard<std::mutex> lock(active->execution_mutex);
        active->backend_execution.emplace(std::move(retry_execution));
      }
      active->session.restore_completion(retry_outcome);
    };

  SequenceGoalTransportState transport_state = SequenceGoalTransportState::kOther;
  try {
    transport_state = active->transport->state();
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger_, "Could not query %s transport state: %s",
      action_label_.c_str(), error.what());
    restore_for_retry(std::move(execution));
    return;
  } catch (...) {
    RCLCPP_WARN(
      logger_, "Could not query %s transport state: unknown exception",
      action_label_.c_str());
    restore_for_retry(std::move(execution));
    return;
  }

  const auto terminal_error = execution.result.outcome.error_code();
  if (terminal_error == TaskErrorCode::kCanceled &&
    transport_state != SequenceGoalTransportState::kCanceling)
  {
    restore_for_retry(std::move(execution));
    return;
  }
  if (terminal_error != TaskErrorCode::kCanceled &&
    transport_state != SequenceGoalTransportState::kExecuting)
  {
    restore_for_retry(std::move(execution));
    return;
  }

  publish_terminal_feedback(active, execution.result.outcome);
  try {
    active->transport->deliver(
      terminal_kind(execution.result.outcome), execution,
      Clock::now() - active->started_at);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "Could not deliver %s result: %s", action_label_.c_str(), error.what());
    restore_for_retry(std::move(execution));
    return;
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "Could not deliver %s result: unknown exception", action_label_.c_str());
    restore_for_retry(std::move(execution));
    return;
  }

  if (!active->session.mark_delivered()) {
    RCLCPP_ERROR(
      logger_, "%s session rejected terminal delivery state", action_label_.c_str());
    return;
  }
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_ == active && active->goal_id == reserved_goal_id_) {
    terminal_delivered_ = true;
  }
}

}  // namespace manipulation::workflow::detail
