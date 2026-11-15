// Responsibility: Implements race-safe stop, pause, worker and completion transitions for one goal.

#include "manipulation/workflow/detail/transfer_sequence_session.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::workflow::detail
{

bool TransferSequenceSession::request_stop(const TaskStopCause cause)
{
  if (cause == TaskStopCause::kNone) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == TransferSequenceSessionPhase::kTerminalReady ||
    phase_ == TransferSequenceSessionPhase::kDelivered)
  {
    return false;
  }
  if (stop_cause_ != TaskStopCause::kNone) {
    return stop_cause_ == cause;
  }
  stop_cause_ = cause;
  return true;
}

TaskStopCause TransferSequenceSession::stop_cause() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return stop_cause_;
}

std::optional<TaskStopCause> TransferSequenceSession::claim_stop_dispatch()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (stop_cause_ == TaskStopCause::kNone || stop_dispatch_claimed_) {
    return std::nullopt;
  }
  stop_dispatch_claimed_ = true;
  return stop_cause_;
}

bool TransferSequenceSession::dispatch_stop(
  const std::function<void(TaskStopCause)> & callback)
{
  const auto cause = claim_stop_dispatch();
  if (!cause) {
    return false;
  }
  if (callback) {
    callback(*cause);
  }
  return true;
}

bool TransferSequenceSession::set_pause_requested(const bool paused)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == TransferSequenceSessionPhase::kTerminalReady ||
    phase_ == TransferSequenceSessionPhase::kDelivered)
  {
    return false;
  }
  pause_requested_ = paused;
  return true;
}

bool TransferSequenceSession::pause_requested() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return pause_requested_;
}

bool TransferSequenceSession::mark_worker_starting()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != TransferSequenceSessionPhase::kReserved) {
    return false;
  }
  phase_ = TransferSequenceSessionPhase::kStarting;
  worker_started_ = true;
  return true;
}

bool TransferSequenceSession::mark_worker_running()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != TransferSequenceSessionPhase::kStarting) {
    return false;
  }
  phase_ = TransferSequenceSessionPhase::kRunning;
  return true;
}

bool TransferSequenceSession::mark_backend_invoked()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if ((phase_ != TransferSequenceSessionPhase::kStarting &&
    phase_ != TransferSequenceSessionPhase::kRunning) ||
    stop_cause_ != TaskStopCause::kNone || backend_invoked_)
  {
    return false;
  }
  backend_invoked_ = true;
  return true;
}

TaskOperationGateResult TransferSequenceSession::run_operation(
  const std::chrono::steady_clock::time_point deadline,
  const std::function<void()> & operation)
{
  if (!operation) {
    throw std::invalid_argument("transfer-sequence session operation must not be empty");
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  if ((phase_ != TransferSequenceSessionPhase::kStarting &&
    phase_ != TransferSequenceSessionPhase::kRunning) || !backend_invoked_ ||
    stop_cause_ != TaskStopCause::kNone)
  {
    return TaskOperationGateResult::kStopped;
  }
  if (std::chrono::steady_clock::now() >= deadline) {
    stop_cause_ = TaskStopCause::kDeadlineExceeded;
    return TaskOperationGateResult::kStopped;
  }
  if (pause_requested_) {
    return TaskOperationGateResult::kPaused;
  }

  operation();
  return TaskOperationGateResult::kExecuted;
}

TaskOperationGateResult TransferSequenceSession::run_maintenance_operation(
  const std::chrono::steady_clock::time_point deadline,
  const std::function<void()> & operation)
{
  if (!operation) {
    throw std::invalid_argument(
            "transfer-sequence maintenance operation must not be empty");
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  if ((phase_ != TransferSequenceSessionPhase::kStarting &&
    phase_ != TransferSequenceSessionPhase::kRunning) || !backend_invoked_ ||
    stop_cause_ != TaskStopCause::kNone)
  {
    return TaskOperationGateResult::kStopped;
  }
  if (std::chrono::steady_clock::now() >= deadline) {
    stop_cause_ = TaskStopCause::kDeadlineExceeded;
    return TaskOperationGateResult::kStopped;
  }

  operation();
  return TaskOperationGateResult::kExecuted;
}

bool TransferSequenceSession::queue_completion(manipulation::task::TaskOutcome outcome)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == TransferSequenceSessionPhase::kTerminalReady ||
    phase_ == TransferSequenceSessionPhase::kDelivered || pending_outcome_)
  {
    return false;
  }
  phase_ = TransferSequenceSessionPhase::kTerminalReady;
  pause_requested_ = false;
  pending_outcome_.emplace(std::move(outcome));
  return true;
}

std::optional<manipulation::task::TaskOutcome> TransferSequenceSession::take_completion()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != TransferSequenceSessionPhase::kTerminalReady ||
    delivery_in_progress_ || !pending_outcome_)
  {
    return std::nullopt;
  }
  delivery_in_progress_ = true;
  std::optional<manipulation::task::TaskOutcome> result;
  result.emplace(std::move(*pending_outcome_));
  pending_outcome_.reset();
  return result;
}

void TransferSequenceSession::restore_completion(manipulation::task::TaskOutcome outcome)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != TransferSequenceSessionPhase::kTerminalReady ||
    !delivery_in_progress_ || pending_outcome_)
  {
    throw std::logic_error("no transfer-sequence completion delivery is in progress");
  }
  pending_outcome_.emplace(std::move(outcome));
  delivery_in_progress_ = false;
}

bool TransferSequenceSession::mark_delivered()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != TransferSequenceSessionPhase::kTerminalReady ||
    !delivery_in_progress_ || pending_outcome_)
  {
    return false;
  }
  phase_ = TransferSequenceSessionPhase::kDelivered;
  delivery_in_progress_ = false;
  return true;
}

TransferSequenceSessionSnapshot TransferSequenceSession::snapshot() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return TransferSequenceSessionSnapshot{
    phase_, stop_cause_, pause_requested_, worker_started_, backend_invoked_,
    pending_outcome_.has_value(), delivery_in_progress_, stop_dispatch_claimed_};
}

}  // namespace manipulation::workflow::detail
