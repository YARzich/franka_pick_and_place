// Responsibility: Models thread-safe lifecycle, stop dispatch and completion delivery for one goal.

#ifndef MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_SESSION_HPP_
#define MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_SESSION_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/task_operation_gate.hpp"
#include "manipulation/workflow/task_stop_cause.hpp"

namespace manipulation::workflow::detail
{

/// Lifecycle phase independent from ROS action transport and planner backend.
enum class TransferSequenceSessionPhase : std::uint8_t
{
  kReserved,
  kStarting,
  kRunning,
  kTerminalReady,
  kDelivered,
};

/// Coherent diagnostic view captured under the session mutex.
struct TransferSequenceSessionSnapshot
{
  TransferSequenceSessionPhase phase{TransferSequenceSessionPhase::kReserved};
  TaskStopCause stop_cause{TaskStopCause::kNone};
  bool pause_requested{false};
  bool worker_started{false};
  bool backend_invoked{false};
  bool completion_ready{false};
  bool delivery_in_progress{false};
  bool stop_dispatch_claimed{false};
};

/**
 * Pure first-writer-wins state machine for one ordered-transfer goal.
 *
 * Stop admission and backend completion share one mutex. If completion records
 * first, a later cancel is rejected; if cancel/deadline records first, the
 * terminal outcome is normalized to that cause. claim_stop_dispatch() returns
 * the cause while releasing the mutex, so user request_stop callbacks always
 * run outside lifecycle locks and can safely re-enter backend-owned state.
 */
class TransferSequenceSession final
{
public:
  TransferSequenceSession() = default;
  ~TransferSequenceSession() = default;

  TransferSequenceSession(const TransferSequenceSession &) = delete;
  TransferSequenceSession & operator=(const TransferSequenceSession &) = delete;
  TransferSequenceSession(TransferSequenceSession &&) = delete;
  TransferSequenceSession & operator=(TransferSequenceSession &&) = delete;

  /// Record the first non-none stop cause; reject after terminal readiness.
  [[nodiscard]] bool request_stop(TaskStopCause cause);

  /// Return the first stop cause without changing it.
  [[nodiscard]] TaskStopCause stop_cause() const;

  /// Claim exactly one external request_stop dispatch for the recorded cause.
  [[nodiscard]] std::optional<TaskStopCause> claim_stop_dispatch();

  /** Invoke one request_stop callback after releasing the session mutex. */
  [[nodiscard]] bool dispatch_stop(
    const std::function<void(TaskStopCause)> & callback);

  /// Store cooperative pause intent before or during worker execution.
  [[nodiscard]] bool set_pause_requested(bool paused);

  /// Return current cooperative pause intent.
  [[nodiscard]] bool pause_requested() const;

  /// Mark that a managed worker start has been claimed for this goal.
  [[nodiscard]] bool mark_worker_starting();

  /// Mark successful worker construction; false means no start was claimed.
  [[nodiscard]] bool mark_worker_running();

  /// Mark entry into backend code; false when sticky stop forbids invocation.
  [[nodiscard]] bool mark_backend_invoked();

  /**
   * Run one short operation under the same mutex as pause and stop admission.
   *
   * Deadline expiry is recorded while the mutex is held. The operation must
   * not block or call back into this session.
   */
  [[nodiscard]] TaskOperationGateResult run_operation(
    std::chrono::steady_clock::time_point deadline,
    const std::function<void()> & operation);

  /**
   * Run short lease maintenance under stop/deadline gate, ignoring pause.
   *
   * The operation must obey the same nonblocking and non-reentrant constraints
   * as run_operation().
   */
  [[nodiscard]] TaskOperationGateResult run_maintenance_operation(
    std::chrono::steady_clock::time_point deadline,
    const std::function<void()> & operation);

  /// Atomically record exactly one backend/pre-start terminal outcome.
  [[nodiscard]] bool queue_completion(manipulation::task::TaskOutcome outcome);

  /// Claim the queued outcome for ROS terminal delivery.
  [[nodiscard]] std::optional<manipulation::task::TaskOutcome> take_completion();

  /// Restore an outcome when the required ROS terminal state is not ready yet.
  void restore_completion(manipulation::task::TaskOutcome outcome);

  /// Mark successful exactly-once ROS terminal delivery.
  [[nodiscard]] bool mark_delivered();

  /// Capture lifecycle, stop, pause, worker, and delivery state atomically.
  [[nodiscard]] TransferSequenceSessionSnapshot snapshot() const;

private:
  mutable std::mutex mutex_;
  TransferSequenceSessionPhase phase_{TransferSequenceSessionPhase::kReserved};
  TaskStopCause stop_cause_{TaskStopCause::kNone};
  bool pause_requested_{false};
  bool worker_started_{false};
  bool backend_invoked_{false};
  bool delivery_in_progress_{false};
  bool stop_dispatch_claimed_{false};
  std::optional<manipulation::task::TaskOutcome> pending_outcome_;
};

}  // namespace manipulation::workflow::detail

#endif  // MANIPULATION__WORKFLOW__DETAIL__TRANSFER_SEQUENCE_SESSION_HPP_
