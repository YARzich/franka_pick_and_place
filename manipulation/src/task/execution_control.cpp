// Responsibility: Implements the synchronized pause/cancel checkpoints used by task workers.

#include "manipulation/task/execution_control.hpp"

#include <algorithm>

namespace manipulation::task
{

void ExecutionControl::request_cancel()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    cancellation_requested_ = true;
  }
  condition_.notify_all();
}

void ExecutionControl::request_pause()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  pause_requested_ = true;
}

void ExecutionControl::resume()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_requested_ = false;
  }
  condition_.notify_all();
}

bool ExecutionControl::cancellation_requested() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return cancellation_requested_;
}

bool ExecutionControl::pause_requested() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return pause_requested_;
}

ExecutionControlSnapshot ExecutionControl::snapshot() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return ExecutionControlSnapshot{
    cancellation_requested_, pause_requested_, paused_at_checkpoint_};
}

CheckpointResult ExecutionControl::checkpoint()
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (cancellation_requested_) {
    return CheckpointResult::kCanceled;
  }

  if (pause_requested_) {
    paused_at_checkpoint_ = true;
    condition_.notify_all();
    condition_.wait(
      lock, [this]() {return cancellation_requested_ || !pause_requested_;});
    paused_at_checkpoint_ = false;
  }

  return cancellation_requested_ ? CheckpointResult::kCanceled :
         CheckpointResult::kContinue;
}

WaitResult ExecutionControl::wait_for(Duration duration)
{
  const auto now = Clock::now();
  if (duration <= Duration::zero()) {
    return wait_until(now);
  }

  const auto maximum_duration = TimePoint::max() - now;
  const auto deadline = duration >= maximum_duration ? TimePoint::max() : now + duration;
  return wait_until(deadline);
}

WaitResult ExecutionControl::wait_until(TimePoint deadline)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (cancellation_requested_) {
    return WaitResult::kCanceled;
  }

  const bool canceled = condition_.wait_until(
    lock, deadline, [this]() {return cancellation_requested_;});
  return canceled ? WaitResult::kCanceled : WaitResult::kCompleted;
}

}  // namespace manipulation::task
