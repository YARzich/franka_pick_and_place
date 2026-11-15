// Responsibility: Implements ordered-transfer checkpoints, deadline waits and feedback forwarding.

#include "manipulation/workflow/transfer_sequence_task_context.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "manipulation/workflow/detail/transfer_sequence_task_context_factory.hpp"

namespace manipulation::workflow
{

TransferSequenceTaskContext detail::TransferSequenceTaskContextFactory::create(
  manipulation::task::ExecutionControl & control,
  const TransferSequenceTaskContext::TimePoint deadline,
  TransferSequenceTaskContext::FeedbackSink feedback_sink,
  TransferSequenceTaskContext::StopCauseProvider stop_cause_provider,
  TransferSequenceTaskContext::DeadlineCallback deadline_callback,
  TransferSequenceTaskContext::OperationGate operation_gate,
  TransferSequenceTaskContext::OperationGate maintenance_operation_gate)
{
  if (!feedback_sink) {
    throw std::invalid_argument("TransferSequenceTaskContext requires a feedback sink");
  }
  if (!stop_cause_provider) {
    throw std::invalid_argument(
            "TransferSequenceTaskContext requires a stop-cause provider");
  }
  if (!deadline_callback) {
    throw std::invalid_argument(
            "TransferSequenceTaskContext requires a deadline callback");
  }
  if (!operation_gate) {
    throw std::invalid_argument(
            "TransferSequenceTaskContext requires an operation gate");
  }
  if (!maintenance_operation_gate) {
    throw std::invalid_argument(
            "TransferSequenceTaskContext requires a maintenance operation gate");
  }
  return TransferSequenceTaskContext(
    control, deadline, std::move(feedback_sink), std::move(stop_cause_provider),
    std::move(deadline_callback), std::move(operation_gate),
    std::move(maintenance_operation_gate));
}

TransferSequenceTaskContext::TransferSequenceTaskContext(
  manipulation::task::ExecutionControl & control,
  const TimePoint deadline,
  FeedbackSink feedback_sink,
  StopCauseProvider stop_cause_provider,
  DeadlineCallback deadline_callback,
  OperationGate operation_gate,
  OperationGate maintenance_operation_gate)
: control_(control),
  deadline_(deadline),
  feedback_sink_(std::move(feedback_sink)),
  stop_cause_provider_(std::move(stop_cause_provider)),
  deadline_callback_(std::move(deadline_callback)),
  operation_gate_(std::move(operation_gate)),
  maintenance_operation_gate_(std::move(maintenance_operation_gate))
{
}

manipulation::task::ExecutionControl &
TransferSequenceTaskContext::control() noexcept
{
  return control_;
}

CooperativeTaskResult TransferSequenceTaskContext::checkpoint()
{
  if (stop_requested()) {
    return CooperativeTaskResult::kStopped;
  }
  if (control_.checkpoint() == manipulation::task::CheckpointResult::kCanceled) {
    return CooperativeTaskResult::kStopped;
  }
  if (Clock::now() >= deadline_) {
    deadline_callback_();
    return CooperativeTaskResult::kStopped;
  }
  return stop_requested() ? CooperativeTaskResult::kStopped :
         CooperativeTaskResult::kContinue;
}

CooperativeTaskResult TransferSequenceTaskContext::wait_for(
  const Duration duration)
{
  const auto now = Clock::now();
  if (duration <= Duration::zero()) {
    return wait_until(now);
  }
  const auto maximum_duration = TimePoint::max() - now;
  return wait_until(duration >= maximum_duration ? TimePoint::max() : now + duration);
}

CooperativeTaskResult TransferSequenceTaskContext::wait_until(
  const TimePoint requested_deadline)
{
  if (stop_requested()) {
    return CooperativeTaskResult::kStopped;
  }

  const auto now = Clock::now();
  if (now >= deadline_) {
    deadline_callback_();
    return CooperativeTaskResult::kStopped;
  }

  const auto bounded_deadline = std::min(requested_deadline, deadline_);
  if (control_.wait_until(bounded_deadline) == manipulation::task::WaitResult::kCanceled) {
    return CooperativeTaskResult::kStopped;
  }
  if (stop_requested()) {
    return CooperativeTaskResult::kStopped;
  }
  if (bounded_deadline == deadline_ && Clock::now() >= deadline_) {
    deadline_callback_();
    return CooperativeTaskResult::kStopped;
  }
  return CooperativeTaskResult::kContinue;
}

CooperativeTaskResult TransferSequenceTaskContext::heartbeat()
{
  if (stop_requested()) {
    return CooperativeTaskResult::kStopped;
  }
  if (Clock::now() >= deadline_) {
    deadline_callback_();
    return CooperativeTaskResult::kStopped;
  }
  return stop_requested() ? CooperativeTaskResult::kStopped :
         CooperativeTaskResult::kContinue;
}

bool TransferSequenceTaskContext::stop_requested() const
{
  return stop_cause() != TaskStopCause::kNone ||
         control_.cancellation_requested();
}

TaskStopCause TransferSequenceTaskContext::stop_cause() const
{
  return stop_cause_provider_();
}

TransferSequenceTaskContext::TimePoint
TransferSequenceTaskContext::deadline() const noexcept
{
  return deadline_;
}

TransferSequenceTaskContext::Duration
TransferSequenceTaskContext::remaining_time() const
{
  const auto now = Clock::now();
  return now >= deadline_ ? Duration::zero() : deadline_ - now;
}

TaskOperationGateResult TransferSequenceTaskContext::run_operation(
  const Operation & operation)
{
  if (!operation) {
    throw std::invalid_argument("TransferSequence operation must not be empty");
  }
  return operation_gate_(operation);
}

TaskOperationGateResult TransferSequenceTaskContext::run_maintenance_operation(
  const Operation & operation)
{
  if (!operation) {
    throw std::invalid_argument(
            "TransferSequence maintenance operation must not be empty");
  }
  return maintenance_operation_gate_(operation);
}

void TransferSequenceTaskContext::report_feedback(
  TransferSequenceFeedbackSnapshot feedback)
{
  feedback_sink_(std::move(feedback));
}

manipulation::task::TaskOutcome
TransferSequenceTaskContext::make_stopped_outcome(
  const manipulation::task::ObjectDisposition disposition,
  const std::uint16_t recovery_count,
  std::string detail) const
{
  using manipulation::task::TaskErrorCode;
  using manipulation::task::TaskOutcome;

  switch (stop_cause()) {
    case TaskStopCause::kClientCanceled:
      return TaskOutcome::make_failure(
        TaskErrorCode::kCanceled, disposition, recovery_count,
        detail.empty() ? "sequence canceled by action client" : std::move(detail));
    case TaskStopCause::kDeadlineExceeded:
      return TaskOutcome::make_failure(
        TaskErrorCode::kTimeout, disposition, recovery_count,
        detail.empty() ? "sequence exceeded its steady-clock deadline" : std::move(detail));
    case TaskStopCause::kShutdown:
      return TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, disposition, recovery_count,
        detail.empty() ? "ordered-transfer sequence server is shutting down" :
        std::move(detail));
    case TaskStopCause::kNone:
      return TaskOutcome::make_failure(
        TaskErrorCode::kInternalError, disposition, recovery_count,
        "make_stopped_outcome called without a sequence stop cause");
  }
  return TaskOutcome::make_failure(
    TaskErrorCode::kInternalError, disposition, recovery_count,
    "unknown ordered-transfer sequence stop cause");
}

}  // namespace manipulation::workflow
