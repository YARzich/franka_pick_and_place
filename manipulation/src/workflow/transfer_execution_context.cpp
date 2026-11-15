// Responsibility: Implements saturating local waits for the transfer execution context port.

#include "manipulation/workflow/transfer_execution_context.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::workflow
{

TransferExecutionControlResult TransferExecutionContext::heartbeat()
{
  return stop_requested() ? TransferExecutionControlResult::kStopped :
         TransferExecutionControlResult::kContinue;
}

TaskOperationGateResult TransferExecutionContext::run_operation(
  const std::function<void()> & operation)
{
  if (!operation) {
    throw std::invalid_argument("transfer execution operation must not be empty");
  }
  if (checkpoint() == TransferExecutionControlResult::kStopped) {
    return TaskOperationGateResult::kStopped;
  }
  operation();
  return TaskOperationGateResult::kExecuted;
}

TaskOperationGateResult TransferExecutionContext::run_maintenance_operation(
  const std::function<void()> & operation)
{
  if (!operation) {
    throw std::invalid_argument("transfer maintenance operation must not be empty");
  }
  if (heartbeat() == TransferExecutionControlResult::kStopped) {
    return TaskOperationGateResult::kStopped;
  }
  operation();
  return TaskOperationGateResult::kExecuted;
}

TaskStopCause TransferExecutionContext::stop_cause() const noexcept
{
  return stop_requested() ? TaskStopCause::kClientCanceled : TaskStopCause::kNone;
}

TransferExecutionControlResult TransferExecutionContext::wait_for(const Duration duration)
{
  const auto now = Clock::now();
  if (duration <= Duration::zero()) {
    return wait_until(now);
  }
  const auto maximum_duration = TimePoint::max() - now;
  return wait_until(duration >= maximum_duration ? TimePoint::max() : now + duration);
}

void TransferExecutionContext::report_waiting(
  const TransferWaitingReason,
  manipulation::task::TaskStateSnapshot feedback)
{
  report_feedback(std::move(feedback));
}

}  // namespace manipulation::workflow
