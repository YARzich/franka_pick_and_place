// Responsibility: Deliver a goal's cancellation to its synchronous planning operation.

#include "goal_planning_cancellation.hpp"

#include <chrono>
#include <exception>
#include <thread>
#include <utility>

namespace manipulation::moveit::detail
{

GoalPlanningCancellation::GoalPlanningCancellation(GoalId goal_id)
: goal_id_(std::move(goal_id)) {}

bool GoalPlanningCancellation::request(const GoalId & goal_id)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (goal_id != goal_id_ || terminal_committed_) {
      return false;
    }
    requested_ = true;
  }
  condition_.notify_all();
  return true;
}

void GoalPlanningCancellation::request_shutdown()
{
  request(goal_id_);
}

bool GoalPlanningCancellation::requested() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return requested_;
}

bool GoalPlanningCancellation::run(
  const std::function<void()> & plan, const std::function<void()> & terminate)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (requested_ || terminal_committed_) {
      return false;
    }
  }
  bool finished = false;
  std::exception_ptr termination_error;
  std::thread watcher([&]() {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [&]() {return requested_ || finished;});
      while (!finished) {
        lock.unlock();
        try {
          // MoveIt 2.12 terminate() reaches only contexts that already exist.
          // Retry while this exact goal still owns generatePlan(), including
          // the request-adapter interval before a context has been created.
          terminate();
        } catch (...) {
          if (!termination_error) {
            termination_error = std::current_exception();
          }
        }
        lock.lock();
        condition_.wait_for(lock, std::chrono::milliseconds{5}, [&]() {return finished;});
      }
    });
  std::exception_ptr planning_error;
  try {
    plan();
  } catch (...) {
    planning_error = std::current_exception();
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    finished = true;
  }
  condition_.notify_all();
  watcher.join();
  if (planning_error) {
    std::rethrow_exception(planning_error);
  }
  if (termination_error) {
    std::rethrow_exception(termination_error);
  }
  return true;
}

bool GoalPlanningCancellation::commit_terminal()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  terminal_committed_ = true;
  return requested_;
}

}  // namespace manipulation::moveit::detail
