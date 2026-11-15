// Responsibility: Owns one background task thread with pause, cancellation and completion
//   snapshots.

#ifndef MANIPULATION__WORKFLOW__SINGLE_GOAL_TASK_WORKER_HPP_
#define MANIPULATION__WORKFLOW__SINGLE_GOAL_TASK_WORKER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "manipulation/task/execution_control.hpp"
#include "manipulation/task/task_status.hpp"

namespace manipulation::workflow
{

/// Observable lifecycle of the single managed task slot.
enum class TaskWorkerState : std::uint8_t
{
  kIdle,
  kStarting,
  kRunning,
  kCompleting,
  kCompleted,
};

/// Result of attempting to occupy the single task slot.
enum class TaskStartResult : std::uint8_t
{
  kStarted,
  kBusy,
};

/// Thread-safe diagnostic view of the managed task worker.
struct TaskWorkerSnapshot
{
  TaskWorkerState state{TaskWorkerState::kIdle};
  bool owned_thread_joinable{false};
  bool control_available{false};
  bool completion_callback_failed{false};
  std::uint64_t generation{0U};
  manipulation::task::ExecutionControlSnapshot execution_control;
};

/**
 * Own exactly one cooperative task thread for a single-goal action server.
 *
 * The class is ROS-independent and is shared by long-running task backends. Each
 * accepted start creates a fresh ExecutionControl and one owned, joinable
 * thread. Threads are never detached. A completed thread is joined before the
 * next task is accepted.
 *
 * Thread safety: start(), control requests, and snapshot() may be called from
 * different threads. The first concurrent start atomically reserves the task
 * slot; all other starts return kBusy. The completion callback is attempted
 * exactly once and always runs without the worker mutex held.
 *
 * Lifetime: the owner must outlive the task and completion callback. In
 * particular, neither callable may destroy this object. That unsupported
 * self-destruction cannot satisfy both mandatory join and the no-detach rule.
 * Calling start() from either callable is safe and returns kBusy.
 */
class SingleGoalTaskWorker final
{
public:
  using TaskCallable = std::function<manipulation::task::TaskOutcome(
        manipulation::task::ExecutionControl &)>;
  using CompletionCallback = std::function<void(manipulation::task::TaskOutcome)>;

  SingleGoalTaskWorker() = default;
  ~SingleGoalTaskWorker();

  SingleGoalTaskWorker(const SingleGoalTaskWorker &) = delete;
  SingleGoalTaskWorker & operator=(const SingleGoalTaskWorker &) = delete;
  SingleGoalTaskWorker(SingleGoalTaskWorker &&) = delete;
  SingleGoalTaskWorker & operator=(SingleGoalTaskWorker &&) = delete;

  /**
   * Start one task and arrange exactly one completion callback invocation.
   *
   * The task receives its per-run ExecutionControl and returns a typed outcome.
   * Its control is published atomically with kStarting, so cancel/pause requests
   * cannot be lost while a previous completed thread is being joined.
   * std::exception and unknown exceptions are converted to kInternalError with
   * kUnknown object disposition before invoking completion. Empty callables
   * throw std::invalid_argument. Thread-creation failures propagate unchanged.
   */
  [[nodiscard]] TaskStartResult start(
    TaskCallable task, CompletionCallback completion_callback);

  /// Request cancellation of the running task; return false when no task runs.
  [[nodiscard]] bool request_cancel();

  /// Request pause at the running task's next cooperative checkpoint.
  [[nodiscard]] bool request_pause();

  /// Resume a pending or acknowledged pause for the running task.
  [[nodiscard]] bool resume();

  /// Capture lifecycle, ownership, callback, and execution-control state.
  [[nodiscard]] TaskWorkerSnapshot snapshot() const;

private:
  using ExecutionControl = manipulation::task::ExecutionControl;

  [[nodiscard]] std::shared_ptr<ExecutionControl> running_control() const;

  mutable std::mutex mutex_;
  std::thread worker_;
  std::shared_ptr<ExecutionControl> control_;
  TaskWorkerState state_{TaskWorkerState::kIdle};
  bool completion_callback_failed_{false};
  std::uint64_t generation_{0U};
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__SINGLE_GOAL_TASK_WORKER_HPP_
