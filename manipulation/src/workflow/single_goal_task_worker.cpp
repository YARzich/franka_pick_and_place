// Responsibility: Implements exception-safe worker startup, cooperative control and thread joining.

#include "manipulation/workflow/single_goal_task_worker.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace manipulation::workflow
{
namespace
{

manipulation::task::TaskOutcome internal_error(const std::string & detail)
{
  return manipulation::task::TaskOutcome::make_failure(
    manipulation::task::TaskErrorCode::kInternalError,
    manipulation::task::ObjectDisposition::kUnknown,
    0U,
    detail);
}

}  // namespace

SingleGoalTaskWorker::~SingleGoalTaskWorker()
{
  std::shared_ptr<ExecutionControl> control;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    control = control_;
  }
  if (control) {
    control->request_cancel();
  }

  if (worker_.joinable()) {
    // Destruction from the owned thread violates the documented lifetime contract.
    if (worker_.get_id() == std::this_thread::get_id()) {
      std::terminate();
    }
    worker_.join();
  }
}

TaskStartResult SingleGoalTaskWorker::start(
  TaskCallable task, CompletionCallback completion_callback)
{
  if (!task || !completion_callback) {
    throw std::invalid_argument("task and completion callback must be callable");
  }

  // Allocate before reserving the slot so allocation failure cannot strand kStarting.
  const auto control = std::make_shared<ExecutionControl>();
  std::thread completed_worker;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != TaskWorkerState::kIdle && state_ != TaskWorkerState::kCompleted) {
      return TaskStartResult::kBusy;
    }

    if (worker_.joinable()) {
      // A worker cannot reach kCompleted until both user callables have returned.
      if (worker_.get_id() == std::this_thread::get_id()) {
        return TaskStartResult::kBusy;
      }
      completed_worker = std::move(worker_);
    }

    // Reserve the slot and publish its control in one critical section.
    control_ = control;
    completion_callback_failed_ = false;
    state_ = TaskWorkerState::kStarting;
  }

  // Never join under mutex: user completion code may inspect this worker.
  if (completed_worker.joinable()) {
    completed_worker.join();
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;
    state_ = TaskWorkerState::kRunning;

    try {
      worker_ = std::thread(
        [this, control, task = std::move(task),
        completion_callback = std::move(completion_callback)]() mutable
        {
          manipulation::task::TaskOutcome outcome = [&]() {
            try {
              return task(*control);
            } catch (const std::exception & error) {
              return internal_error(
                  std::string{"task callable threw std::exception: "} + error.what());
            } catch (...) {
              return internal_error("task callable threw an unknown exception");
            }
          }();

          {
            const std::lock_guard<std::mutex> worker_lock(mutex_);
            state_ = TaskWorkerState::kCompleting;
          }

          bool callback_failed = false;
          try {
            completion_callback(std::move(outcome));
          } catch (...) {
            // Retrying would violate exactly-once delivery; diagnostics expose the failure.
            callback_failed = true;
          }

          {
            const std::lock_guard<std::mutex> worker_lock(mutex_);
            completion_callback_failed_ = callback_failed;
            state_ = TaskWorkerState::kCompleted;
          }
        });
    } catch (...) {
      control_.reset();
      state_ = TaskWorkerState::kIdle;
      throw;
    }
  }

  return TaskStartResult::kStarted;
}

bool SingleGoalTaskWorker::request_cancel()
{
  const auto control = running_control();
  if (!control) {
    return false;
  }
  control->request_cancel();
  return true;
}

bool SingleGoalTaskWorker::request_pause()
{
  const auto control = running_control();
  if (!control) {
    return false;
  }
  control->request_pause();
  return true;
}

bool SingleGoalTaskWorker::resume()
{
  const auto control = running_control();
  if (!control) {
    return false;
  }
  control->resume();
  return true;
}

TaskWorkerSnapshot SingleGoalTaskWorker::snapshot() const
{
  TaskWorkerSnapshot result;
  std::shared_ptr<ExecutionControl> control;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    result.state = state_;
    result.owned_thread_joinable = worker_.joinable();
    result.control_available = static_cast<bool>(control_);
    result.completion_callback_failed = completion_callback_failed_;
    result.generation = generation_;
    control = control_;
  }
  if (control) {
    result.execution_control = control->snapshot();
  }
  return result;
}

std::shared_ptr<SingleGoalTaskWorker::ExecutionControl>
SingleGoalTaskWorker::running_control() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != TaskWorkerState::kStarting && state_ != TaskWorkerState::kRunning) {
    return {};
  }
  return control_;
}

}  // namespace manipulation::workflow
