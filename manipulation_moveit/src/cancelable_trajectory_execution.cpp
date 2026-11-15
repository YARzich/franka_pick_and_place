// Responsibility: Serialize one trajectory's admission, completion, exact cancel, and shutdown.

#include "cancelable_trajectory_execution.hpp"

#include <exception>
#include <utility>

namespace manipulation::moveit::detail
{

struct CancelableTrajectoryExecution::Completion
{
  std::mutex mutex;
  std::condition_variable cv;
  std::optional<TrajectoryExecutionStatus> status;
};

CancelableTrajectoryExecution::CancelableTrajectoryExecution()
: worker_([this]() {run();})
{
}

CancelableTrajectoryExecution::~CancelableTrajectoryExecution()
{
  shutdown();
}

bool CancelableTrajectoryExecution::reserve(const TrajectoryGoalId & id)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_ || faulted_ || active_id_) {
    return false;
  }
  active_id_ = id;
  cancel_requested_ = false;
  finishing_ = false;
  return true;
}

bool CancelableTrajectoryExecution::submit(
  const TrajectoryGoalId & id, TrajectoryExecutionOperations operations)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_ || faulted_ || !active_id_ || *active_id_ != id || pending_ || completion_) {
    return false;
  }
  pending_ = Job{id, std::move(operations)};
  cv_.notify_one();
  return true;
}

bool CancelableTrajectoryExecution::request_cancel(const TrajectoryGoalId & id)
{
  std::shared_ptr<Completion> completion;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!active_id_ || *active_id_ != id || finishing_ || faulted_) {
      return false;
    }
    cancel_requested_ = true;
    completion = completion_;
  }
  if (completion) {
    const std::lock_guard<std::mutex> lock(completion->mutex);
    completion->cv.notify_all();
  }
  cv_.notify_all();
  return true;
}

void CancelableTrajectoryExecution::shutdown() noexcept
{
  std::shared_ptr<Completion> completion;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    cancel_requested_ = true;
    completion = completion_;
  }
  if (completion) {
    const std::lock_guard<std::mutex> lock(completion->mutex);
    completion->cv.notify_all();
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool CancelableTrajectoryExecution::stopped() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return shutdown_ || cancel_requested_;
}

void CancelableTrajectoryExecution::run()
{
  for (;; ) {
    std::optional<Job> job;
    bool retiring_fault = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() {return shutdown_ || pending_.has_value();});
      if (!pending_) {
        job = std::move(faulted_job_);
        faulted_job_.reset();
        retiring_fault = true;
      } else {
        job = std::move(pending_);
        pending_.reset();
        completion_ = std::make_shared<Completion>();
      }
    }
    if (retiring_fault) {
      if (job) {
        try {
          job->operations.stop();
          job->operations.join();
        } catch (...) {
          fault(*job, "trajectory stop remains unconfirmed during capability shutdown");
        }
      }
      return;
    }
    execute(*job);
    const std::lock_guard<std::mutex> lock(mutex_);
    if (faulted_) {
      faulted_job_ = std::move(job);
    }
  }
}

void CancelableTrajectoryExecution::execute(Job & job)
{
  const auto completion = completion_;
  bool committed = false;
  try {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      // Linearization point: after committing, push and start are indivisible
      // to cancellation. TEM cannot clear a pushed but never started trajectory.
      committed = !cancel_requested_ && !shutdown_;
    }
    if (!committed) {
      finish(job, {TrajectoryExecutionStatus::kCanceled, "trajectory canceled before dispatch"});
      return;
    }
    if (!job.operations.push()) {
      finish(
        job, {TrajectoryExecutionStatus::kFailed, "trajectory manager rejected the trajectory"});
      return;
    }
    job.operations.start(
      [completion](const TrajectoryExecutionStatus status) {
        const std::lock_guard<std::mutex> lock(completion->mutex);
        completion->status = status;
        completion->cv.notify_all();
      });

    std::optional<TrajectoryExecutionStatus> status;
    {
      std::unique_lock<std::mutex> lock(completion->mutex);
      completion->cv.wait(lock, [this, &completion]() {
          return completion->status.has_value() || stopped();
        });
      status = completion->status;
    }
    if (stopped()) {
      job.operations.stop();
    }
    // Only this worker calls stop/join. In particular it never waits for the
    // TEM execution thread concurrently with another stopExecution caller.
    job.operations.join();
    finish(
      job, {status.value_or(TrajectoryExecutionStatus::kCanceled), "trajectory execution ended"});
  } catch (const std::exception & error) {
    if (committed) {
      try {
        job.operations.stop();
      } catch (...) {
        // Preserve unknown physical/queue state; no terminal action result.
      }
    }
    fault(job, std::string{"trajectory execution state is uncertain: "} + error.what());
  } catch (...) {
    if (committed) {
      try {
        job.operations.stop();
      } catch (...) {
      }
    }
    fault(job, "trajectory execution state is uncertain: unknown exception");
  }
}

void CancelableTrajectoryExecution::finish(Job & job, TrajectoryExecutionOutcome outcome)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    finishing_ = true;
    if (cancel_requested_ || shutdown_) {
      outcome = {
        TrajectoryExecutionStatus::kCanceled, "trajectory canceled; execution stopped", !shutdown_};
    }
  }
  job.operations.finish(outcome);
  const std::lock_guard<std::mutex> lock(mutex_);
  active_id_.reset();
  completion_.reset();
}

void CancelableTrajectoryExecution::fault(Job & job, const std::string & message) noexcept
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    faulted_ = true;
    finishing_ = true;
  }
  try {
    job.operations.fault(message);
  } catch (...) {
  }
}

}  // namespace manipulation::moveit::detail
