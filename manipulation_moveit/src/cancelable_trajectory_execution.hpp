// Responsibility: Own one exact trajectory goal and serialize dispatch, stop, and join.

#ifndef CANCELABLE_TRAJECTORY_EXECUTION_HPP_
#define CANCELABLE_TRAJECTORY_EXECUTION_HPP_

#include <array>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace manipulation::moveit::detail
{

using TrajectoryGoalId = std::array<std::uint8_t, 16>;

enum class TrajectoryExecutionStatus {kSucceeded, kCanceled, kTimedOut, kFailed};

struct TrajectoryExecutionOutcome
{
  TrajectoryExecutionStatus status;
  std::string detail;
  bool cancellation_requested{false};
};

struct TrajectoryExecutionOperations
{
  std::function<bool()> push;
  std::function<void(std::function<void(TrajectoryExecutionStatus)>)> start;
  std::function<void()> stop;
  std::function<void()> join;
  std::function<void(const TrajectoryExecutionOutcome &)> finish;
  std::function<void(const std::string &)> fault;
};

/**
 * One owned worker is the only caller of execution operations. Cancellation only
 * records intent; callbacks remain available while the worker stops and joins.
 * Uncertain dispatch/stop faults permanently close admission and never publish
 * terminal-safe evidence. Completion callbacks own their state independently.
 */
class CancelableTrajectoryExecution final
{
public:
  CancelableTrajectoryExecution();
  ~CancelableTrajectoryExecution();
  CancelableTrajectoryExecution(const CancelableTrajectoryExecution &) = delete;
  CancelableTrajectoryExecution & operator=(const CancelableTrajectoryExecution &) = delete;

  [[nodiscard]] bool reserve(const TrajectoryGoalId & id);
  [[nodiscard]] bool submit(const TrajectoryGoalId & id, TrajectoryExecutionOperations operations);
  [[nodiscard]] bool request_cancel(const TrajectoryGoalId & id);
  void shutdown() noexcept;

private:
  struct Completion;
  struct Job
  {
    TrajectoryGoalId id;
    TrajectoryExecutionOperations operations;
  };

  [[nodiscard]] bool stopped() const;
  void run();
  void execute(Job & job);
  void finish(Job & job, TrajectoryExecutionOutcome outcome);
  void fault(Job & job, const std::string & message) noexcept;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<TrajectoryGoalId> active_id_;
  std::optional<Job> pending_;
  // Unfinished action handles must stay alive: their destructor can synthesize
  // a terminal response even though stop/dispatch evidence remains unknown.
  std::optional<Job> faulted_job_;
  std::shared_ptr<Completion> completion_;
  bool cancel_requested_{false};
  bool finishing_{false};
  bool faulted_{false};
  bool shutdown_{false};
  std::thread worker_;
};

}  // namespace manipulation::moveit::detail

#endif  // CANCELABLE_TRAJECTORY_EXECUTION_HPP_
