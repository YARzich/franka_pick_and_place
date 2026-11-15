// Responsibility: Defines thread-safe pause, cancellation and recovery control for long-running
//   tasks.

#ifndef MANIPULATION__TASK__EXECUTION_CONTROL_HPP_
#define MANIPULATION__TASK__EXECUTION_CONTROL_HPP_

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace manipulation::task
{

/// Result returned when a task worker reaches a cooperative checkpoint.
enum class CheckpointResult
{
  kContinue,
  kCanceled,
};

/// Result of a steady-clock wait that can be interrupted by cancellation.
enum class WaitResult
{
  kCompleted,
  kCanceled,
};

/// Atomic view of the execution-control flags captured under one lock.
struct ExecutionControlSnapshot
{
  bool cancellation_requested{false};
  bool pause_requested{false};
  bool paused_at_checkpoint{false};
};

/**
 * Cooperative cancellation and safe-point pause control for one task worker.
 *
 * Ownership: create one instance per task/goal. The owner must keep it alive
 * until all worker and control-callback calls have returned. The class is
 * intentionally non-copyable and non-movable.
 *
 * Thread safety: every public operation may be called concurrently. A cancel
 * request is sticky for the lifetime of the object and wakes a worker paused
 * inside checkpoint() or blocked in wait_for()/wait_until().
 *
 * Pause semantics: request_pause() records intent; only checkpoint() enters a
 * paused state. This prevents a workflow from stopping between physical steps
 * that must remain atomic. Core waits do not create implicit checkpoints.
 *
 * Clock semantics: deadlines use std::chrono::steady_clock and are therefore
 * independent from ROS time and simulator clock resets.
 */
class ExecutionControl final
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;

  ExecutionControl() = default;
  ~ExecutionControl() = default;

  ExecutionControl(const ExecutionControl &) = delete;
  ExecutionControl & operator=(const ExecutionControl &) = delete;
  ExecutionControl(ExecutionControl &&) = delete;
  ExecutionControl & operator=(ExecutionControl &&) = delete;

  /// Request irreversible task cancellation and wake blocked core waits.
  void request_cancel();

  /// Request a cooperative pause at the worker's next safe checkpoint.
  void request_pause();

  /// Clear a pending or active pause request and wake a paused checkpoint.
  void resume();

  /// Return true after cancellation has been requested.
  [[nodiscard]] bool cancellation_requested() const;

  /// Return true while a pause is pending or active.
  [[nodiscard]] bool pause_requested() const;

  /// Capture cancellation, pause request and pause acknowledgement together.
  [[nodiscard]] ExecutionControlSnapshot snapshot() const;

  /**
   * Apply pause/cancel requests at a workflow-defined safe boundary.
   *
   * Blocks while pause is requested. A concurrent resume returns kContinue;
   * a concurrent or earlier cancel returns kCanceled and wakes the wait.
   */
  [[nodiscard]] CheckpointResult checkpoint();

  /**
   * Wait for a steady-clock duration unless cancellation is requested.
   *
   * A non-positive duration completes immediately unless cancellation was
   * already requested. Pause requests are intentionally ignored until the
   * caller invokes checkpoint().
   */
  [[nodiscard]] WaitResult wait_for(Duration duration);

  /**
   * Wait until an absolute steady-clock deadline or task cancellation.
   *
   * An expired deadline completes immediately unless cancellation was already
   * requested. This method never observes ROS or system-wall-clock changes.
   */
  [[nodiscard]] WaitResult wait_until(TimePoint deadline);

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool cancellation_requested_{false};
  bool pause_requested_{false};
  bool paused_at_checkpoint_{false};
};

}  // namespace manipulation::task

#endif  // MANIPULATION__TASK__EXECUTION_CONTROL_HPP_
