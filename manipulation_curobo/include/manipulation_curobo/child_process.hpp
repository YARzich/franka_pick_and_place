// Responsibility: Defines RAII ownership and bounded termination of an external planner process
//   group.

#ifndef MANIPULATION_CUROBO__CHILD_PROCESS_HPP_
#define MANIPULATION_CUROBO__CHILD_PROCESS_HPP_

#include <sys/types.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace manipulation_curobo::curobo
{

enum class ProcessOutcome
{
  kExited,
  kSignaled,
  kTimedOut,
  kCanceled,
  kSpawnFailed,
  kWaitFailed,
};

/// Terminal evidence returned after an owned child has been reaped.
struct ProcessResult
{
  ProcessOutcome outcome{ProcessOutcome::kWaitFailed};
  int exit_code{-1};
  int signal_number{0};
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept;
};

/**
 * Own one argv-spawned child process group.
 *
 * start(), wait() and waitUntil() run on the planning worker. requestStop() may run
 * concurrently from MoveIt's cancellation thread; it is non-blocking and
 * signals only this child's process group. waitUntil() uses a steady deadline,
 * performs bounded TERM/KILL escalation, and reaps the child before return.
 * The leader remains waitable until the last group signal, preventing PID reuse
 * from redirecting cleanup. Remaining descendants are killed before reaping the
 * leader, including after a normal exit; its original exit evidence is kept.
 */
class ChildProcess final
{
public:
  using Clock = std::chrono::steady_clock;

  ChildProcess() = default;
  ~ChildProcess() noexcept;

  ChildProcess(const ChildProcess &) = delete;
  ChildProcess & operator=(const ChildProcess &) = delete;
  ChildProcess(ChildProcess &&) = delete;
  ChildProcess & operator=(ChildProcess &&) = delete;

  /// Spawn argv[0] without a shell. Empty arguments and repeated starts fail.
  [[nodiscard]] bool start(const std::vector<std::string> & argv);

  /// Wait to the absolute steady deadline and always reap an accepted child.
  [[nodiscard]] ProcessResult waitUntil(Clock::time_point deadline);

  /**
   * Wait without imposing a supervisor deadline and reap the accepted child.
   *
   * This is used for request workers because MotionGen construction and warmup
   * are not part of MoveIt's allowed_planning_time. The wait remains
   * interruptible through requestStop() and bounded during owner destruction.
   */
  [[nodiscard]] ProcessResult wait();

  /// Request cancellation of this exact process group and wake waitUntil().
  void requestStop() noexcept;

  /// Return the owned PID for diagnostics/tests, or -1 after reap/no start.
  [[nodiscard]] pid_t pid() const noexcept;

private:
  [[nodiscard]] ProcessResult pollExitLocked();
  [[nodiscard]] ProcessResult stopAndReap(
    ProcessOutcome outcome, std::string detail) noexcept;
  void signalGroupLocked(int signal_number) noexcept;
  [[nodiscard]] bool waitForExitUntil(Clock::time_point deadline) noexcept;
  void resetLocked() noexcept;

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  pid_t pid_{-1};
  bool started_{false};
  bool stop_requested_{false};
  bool group_signals_closed_{false};
  int spawn_error_{0};
};

}  // namespace manipulation_curobo::curobo

#endif  // MANIPULATION_CUROBO__CHILD_PROCESS_HPP_
