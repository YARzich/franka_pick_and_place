// Responsibility: Implements fail-closed sequence evidence validation and normalization.

#include "detail/transfer_sequence_execution_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "manipulation/workflow/task_message_adapter.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using ObjectDisposition = manipulation::task::ObjectDisposition;
using ResourceReleaseDisposition = manipulation::task::ResourceReleaseDisposition;
using TaskErrorCode = manipulation::task::TaskErrorCode;
using TaskOutcome = manipulation::task::TaskOutcome;

TransferSequenceExecution outcome_without_steps(TaskOutcome outcome)
{
  return {
    TransferSequenceResult{std::move(outcome), 0U, std::nullopt, {}}, {}};
}

bool same_machine_outcome(const TaskOutcome & lhs, const TaskOutcome & rhs)
{
  return lhs.succeeded() == rhs.succeeded() &&
         lhs.error_code() == rhs.error_code() &&
         lhs.disposition() == rhs.disposition() &&
         lhs.recovery_count() == rhs.recovery_count() &&
         lhs.resource_release() == rhs.resource_release();
}

std::optional<std::string> invalid_outcome_enum(
  const TaskOutcome & outcome,
  const std::string & field)
{
  try {
    (void)to_wire_error_code(outcome.error_code());
    (void)to_wire_disposition(outcome.disposition());
  } catch (const std::invalid_argument &) {
    return field + " contains an unknown domain enum";
  }
  switch (outcome.resource_release()) {
    case ResourceReleaseDisposition::kReleaseSafe:
    case ResourceReleaseDisposition::kQuarantineRequired:
      break;
    default:
      return field + " contains an unknown resource-release enum";
  }
  return std::nullopt;
}

bool execution_requires_quarantine(
  const TransferSequenceExecution & execution) noexcept
{
  if (!execution.result.outcome.resources_release_safe()) {
    return true;
  }
  return std::any_of(
    execution.result.step_outcomes.cbegin(),
    execution.result.step_outcomes.cend(),
    [](const TaskOutcome & outcome) {
      return !outcome.resources_release_safe();
    });
}

std::optional<std::string> execution_contract_error(
  const TransferSequenceExecution & execution,
  const std::size_t requested_step_count)
{
  const auto & result = execution.result;
  const auto & outcomes = result.step_outcomes;
  if (const auto error = invalid_outcome_enum(result.outcome, "aggregate outcome")) {
    return error;
  }
  for (std::size_t index = 0U; index < outcomes.size(); ++index) {
    if (const auto error = invalid_outcome_enum(
        outcomes[index], "step outcome " + std::to_string(index)))
    {
      return error;
    }
  }
  if (outcomes.size() != execution.step_elapsed.size()) {
    return "step_outcomes and step_elapsed sizes differ";
  }
  if (std::any_of(
      execution.step_elapsed.cbegin(), execution.step_elapsed.cend(),
      [](const auto elapsed) {return elapsed < std::chrono::steady_clock::duration::zero();}))
  {
    return "step_elapsed contains a negative duration";
  }
  if (outcomes.size() > requested_step_count) {
    return "backend returned more step outcomes than requested";
  }
  if (result.completed_step_count > outcomes.size()) {
    return "completed_step_count exceeds returned step outcomes";
  }
  if (!std::all_of(
      outcomes.cbegin(), outcomes.cbegin() + result.completed_step_count,
      [](const TaskOutcome & outcome) {
        return outcome.succeeded() &&
               outcome.disposition() == ObjectDisposition::kObjectAtTarget;
      }))
  {
    return "completed sequence prefix lacks a confirmed target disposition";
  }

  if (result.failed_step_index) {
    if (*result.failed_step_index >
      static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max()))
    {
      return "failed_step_index cannot be represented on the action wire";
    }
    if (*result.failed_step_index != result.completed_step_count ||
      outcomes.size() != result.completed_step_count + 1U)
    {
      return "failed step must immediately follow the completed prefix";
    }
    if (outcomes.back().succeeded() || result.outcome.succeeded()) {
      return "failed_step_index accompanies a successful terminal outcome";
    }
    if (!same_machine_outcome(result.outcome, outcomes.back())) {
      return "aggregate failure contradicts the terminal failed step";
    }
  } else {
    if (outcomes.size() != result.completed_step_count) {
      return "step outcome without a failed index follows the completed prefix";
    }
    if (!std::all_of(
        outcomes.cbegin(), outcomes.cend(),
        [](const TaskOutcome & outcome) {return outcome.succeeded();}))
    {
      return "failed step outcome is missing failed_step_index";
    }
    if (!result.outcome.succeeded() && result.completed_step_count > 0U &&
      result.completed_step_count < requested_step_count &&
      result.outcome.disposition() != outcomes.back().disposition())
    {
      return "between-step failure contradicts the last completed disposition";
    }
  }

  if (result.outcome.succeeded() && requested_step_count == 0U) {
    return "successful aggregate cannot represent an empty requested sequence";
  }
  if (result.outcome.succeeded() &&
    (result.completed_step_count != requested_step_count ||
    outcomes.size() != requested_step_count))
  {
    return "successful aggregate does not contain every requested successful step";
  }
  if (result.outcome.succeeded() &&
    (!same_machine_outcome(result.outcome, outcomes.back()) ||
    result.outcome.disposition() != ObjectDisposition::kObjectAtTarget))
  {
    return "successful aggregate lacks matching target postcondition evidence";
  }
  return std::nullopt;
}

TransferSequenceExecution fail_closed_contract(
  std::string reason,
  const std::string_view action_label,
  const ResourceReleaseDisposition resource_release)
{
  return outcome_without_steps(
    TaskOutcome::make_failure(
      TaskErrorCode::kInternalError, ObjectDisposition::kUnknown, 0U,
      "invalid " + std::string{action_label} + " backend result: " +
      std::move(reason), resource_release));
}

TaskOutcome normalize_terminal_outcome(
  TaskOutcome outcome,
  const TaskStopCause cause,
  const std::string_view action_label)
{
  const auto preserve_backend_detail = [&outcome](std::string lifecycle_detail) {
      if (!outcome.detail().empty()) {
        lifecycle_detail += "; ";
        lifecycle_detail += outcome.detail();
      }
      return lifecycle_detail;
    };
  if (cause == TaskStopCause::kClientCanceled) {
    return TaskOutcome::make_failure(
      TaskErrorCode::kCanceled, outcome.disposition(), outcome.recovery_count(),
      preserve_backend_detail(
        std::string{action_label} + " canceled by action client"),
      outcome.resource_release());
  }
  if (cause == TaskStopCause::kDeadlineExceeded) {
    return TaskOutcome::make_failure(
      TaskErrorCode::kTimeout, outcome.disposition(), outcome.recovery_count(),
      preserve_backend_detail(
        std::string{action_label} +
        " exceeded its steady-clock goal deadline"),
      outcome.resource_release());
  }
  if (cause == TaskStopCause::kShutdown) {
    return TaskOutcome::make_failure(
      TaskErrorCode::kInternalError, outcome.disposition(), outcome.recovery_count(),
      preserve_backend_detail(
        std::string{action_label} +
        " server shut down during backend execution"),
      outcome.resource_release());
  }
  if (outcome.error_code() == TaskErrorCode::kCanceled) {
    return TaskOutcome::make_failure(
      TaskErrorCode::kInternalError, outcome.disposition(), outcome.recovery_count(),
      preserve_backend_detail(
        "sequence backend returned CANCELED without an accepted client cancel"),
      outcome.resource_release());
  }
  return outcome;
}

}  // namespace

TransferSequenceExecutionPolicyResult apply_transfer_sequence_execution_policy(
  TransferSequenceExecution execution,
  const std::size_t requested_step_count,
  const TaskStopCause stop_cause,
  const std::string_view action_label)
{
  const auto contract_error = execution_contract_error(execution, requested_step_count);
  if (contract_error) {
    const auto resource_release =
      execution_requires_quarantine(execution) ?
      ResourceReleaseDisposition::kQuarantineRequired :
      ResourceReleaseDisposition::kReleaseSafe;
    execution = fail_closed_contract(
      *contract_error, action_label, resource_release);
  }
  execution.result.outcome = normalize_terminal_outcome(
    std::move(execution.result.outcome), stop_cause, action_label);
  if (stop_cause != TaskStopCause::kNone &&
    execution.result.failed_step_index && !execution.result.step_outcomes.empty())
  {
    execution.result.step_outcomes.back() = execution.result.outcome;
  }
  return {std::move(execution), contract_error};
}

}  // namespace manipulation::workflow::detail
