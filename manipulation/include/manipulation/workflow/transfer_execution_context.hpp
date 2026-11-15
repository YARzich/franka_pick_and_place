// Responsibility: Defines ROS-independent cooperative control and feedback for transfer execution.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_EXECUTION_CONTEXT_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_EXECUTION_CONTEXT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/task_operation_gate.hpp"
#include "manipulation/workflow/task_stop_cause.hpp"

namespace manipulation::workflow
{

/// Result of a pause/cancel/shutdown-aware strategy checkpoint or wait.
enum class TransferExecutionControlResult : std::uint8_t
{
  kContinue,
  kStopped,
};

/// Machine-readable reason for a non-physical workflow wait.
enum class TransferWaitingReason : std::uint8_t
{
  kNone = 0U,
  kObservation = 1U,
  kTransform = 2U,
  kGraspGeneration = 3U,
  kSceneReadiness = 4U,
  kHealthReadiness = 5U,
  kPlanning = 6U,
  kRosEndpoint = 7U,
};

/**
 * Goal-scoped strategy control without ROS ownership or a whole-goal deadline.
 *
 * Implementations apply cooperative pause in checkpoint()/wait_until() and
 * report cancel or shutdown through kStopped. A requested wait is a physical
 * settle/backoff boundary owned by the caller; it is not an implicit task
 * timeout. Physical-command timeouts belong to strategy/backend configuration.
 */
class TransferExecutionContext
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  virtual ~TransferExecutionContext() = default;

  [[nodiscard]] virtual TransferExecutionControlResult checkpoint() = 0;
  [[nodiscard]] virtual bool stop_requested() const noexcept = 0;
  [[nodiscard]] virtual TransferExecutionControlResult wait_until(TimePoint deadline) = 0;

  /**
   * Observe cancel/shutdown and service execution-scoped maintenance.
   *
   * Unlike checkpoint(), this method never applies a cooperative pause. A
   * backend uses it while an already accepted physical command is still
   * active, so goal-scoped leases and diagnostics remain alive without
   * pretending that the command can be paused atomically.
   */
  [[nodiscard]] virtual TransferExecutionControlResult heartbeat();

  /**
   * Linearize one short dispatch against lifecycle pause and stop admission.
   *
   * Production lifecycle adapters override this method with their session
   * operation gate. The default preserves simple fake contexts by applying
   * their checkpoint before invoking the operation.
   */
  [[nodiscard]] virtual TaskOperationGateResult run_operation(
    const std::function<void()> & operation);

  /**
   * Linearize lease maintenance against stop/deadline while ignoring pause.
   *
   * This narrow gate exists so an already admitted goal can renew ownership
   * while paused. It must not be used for planning, physical commands, or
   * ordinary scene mutations.
   */
  [[nodiscard]] virtual TaskOperationGateResult run_maintenance_operation(
    const std::function<void()> & operation);

  /** Return the typed lifecycle cause used to close an active wait. */
  [[nodiscard]] virtual TaskStopCause stop_cause() const noexcept;

  /** Saturating convenience wrapper over the explicit local wait boundary. */
  [[nodiscard]] TransferExecutionControlResult wait_for(Duration duration);

  virtual void report_feedback(manipulation::task::TaskStateSnapshot feedback) = 0;

  /** Report typed non-physical waiting while preserving basic-context compatibility. */
  virtual void report_waiting(
    TransferWaitingReason reason,
    manipulation::task::TaskStateSnapshot feedback);
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_EXECUTION_CONTEXT_HPP_
