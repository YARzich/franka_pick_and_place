// Responsibility: Gives an ordered-transfer backend cooperative control and feedback access.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_TASK_CONTEXT_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_TASK_CONTEXT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "manipulation/task/execution_control.hpp"
#include "manipulation/workflow/task_operation_gate.hpp"
#include "manipulation/workflow/task_stop_cause.hpp"
#include "manipulation/workflow/transfer_sequence_action_types.hpp"

namespace manipulation::workflow
{

namespace detail
{
class TransferSequenceTaskContextFactory;
}

/**
 * Per-goal cooperative API passed to an ordered-transfer backend.
 *
 * The context has no ROS goal handle and is valid only while the execute
 * callback runs on the server-owned worker. A backend passes control() to its
 * pure sequence runner, checks the steady deadline around blocking adapters,
 * and reports runner feedback without retaining the callback.
 */
class TransferSequenceTaskContext final
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;
  using FeedbackSink = std::function<void(TransferSequenceFeedbackSnapshot)>;
  using StopCauseProvider = std::function<TaskStopCause()>;
  using DeadlineCallback = std::function<void()>;
  using Operation = std::function<void()>;
  using OperationGate =
    std::function<TaskOperationGateResult(const Operation &)>;

  TransferSequenceTaskContext(const TransferSequenceTaskContext &) = delete;
  TransferSequenceTaskContext & operator=(const TransferSequenceTaskContext &) = delete;
  TransferSequenceTaskContext(TransferSequenceTaskContext &&) = delete;
  TransferSequenceTaskContext & operator=(TransferSequenceTaskContext &&) = delete;
  ~TransferSequenceTaskContext() = default;

  /// Return the goal-scoped pause/cancel control passed to every sequence step.
  [[nodiscard]] manipulation::task::ExecutionControl & control() noexcept;

  /// Apply pause at a safe sequence boundary or observe cancellation/deadline.
  [[nodiscard]] CooperativeTaskResult checkpoint();

  /// Wait for steady duration, bounded by the outer sequence deadline.
  [[nodiscard]] CooperativeTaskResult wait_for(Duration duration);

  /// Wait until a steady deadline, never beyond the outer sequence deadline.
  [[nodiscard]] CooperativeTaskResult wait_until(TimePoint requested_deadline);

  /**
   * Observe stop/deadline without applying a cooperative pause.
   *
   * This is the control seam for an already accepted physical command whose
   * completion wait must keep running until its next safe boundary.
   */
  [[nodiscard]] CooperativeTaskResult heartbeat();

  /// Return true after client cancel, deadline, shutdown, or worker cancel.
  [[nodiscard]] bool stop_requested() const;

  /// Return the first lifecycle stop cause recorded for this sequence goal.
  [[nodiscard]] TaskStopCause stop_cause() const;

  /// Return the absolute steady-clock deadline shared by all sequence steps.
  [[nodiscard]] TimePoint deadline() const noexcept;

  /// Return non-negative steady time remaining before the outer deadline.
  [[nodiscard]] Duration remaining_time() const;

  /**
   * Linearize one non-blocking operation against pause, stop, and deadline.
   *
   * The operation runs while the goal lifecycle owns its session mutex and
   * must therefore stay short, never wait, and never re-enter that lifecycle.
   */
  [[nodiscard]] TaskOperationGateResult run_operation(
    const Operation & operation);

  /**
   * Linearize lease maintenance against stop/deadline without applying pause.
   *
   * This gate is reserved for keeping already-owned resources alive.
   */
  [[nodiscard]] TaskOperationGateResult run_maintenance_operation(
    const Operation & operation);

  /// Forward sequence and active-step progress to the ROS lifecycle facade.
  void report_feedback(TransferSequenceFeedbackSnapshot feedback);

  /// Build the required cancel/timeout/shutdown outcome after a stopped operation.
  [[nodiscard]] manipulation::task::TaskOutcome make_stopped_outcome(
    manipulation::task::ObjectDisposition disposition,
    std::uint16_t recovery_count = 0U,
    std::string detail = {}) const;

private:
  friend class detail::TransferSequenceTaskContextFactory;

  TransferSequenceTaskContext(
    manipulation::task::ExecutionControl & control,
    TimePoint deadline,
    FeedbackSink feedback_sink,
    StopCauseProvider stop_cause_provider,
    DeadlineCallback deadline_callback,
    OperationGate operation_gate,
    OperationGate maintenance_operation_gate);

  manipulation::task::ExecutionControl & control_;
  TimePoint deadline_;
  FeedbackSink feedback_sink_;
  StopCauseProvider stop_cause_provider_;
  DeadlineCallback deadline_callback_;
  OperationGate operation_gate_;
  OperationGate maintenance_operation_gate_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_SEQUENCE_TASK_CONTEXT_HPP_
