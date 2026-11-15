// Responsibility: Verifies pause, resume, cancellation, and interruptible steady-clock waits.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "manipulation/task/execution_control.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::task::CheckpointResult;
using manipulation::task::ExecutionControl;
using manipulation::task::WaitResult;

bool wait_for_pause_acknowledgement(
  const ExecutionControl & control, ExecutionControl::TimePoint deadline)
{
  while (ExecutionControl::Clock::now() < deadline) {
    if (control.snapshot().paused_at_checkpoint) {
      return true;
    }
    std::this_thread::yield();
  }
  return control.snapshot().paused_at_checkpoint;
}

TEST(ExecutionControl, StartsReadyToRun)
{
  ExecutionControl control;

  const auto snapshot = control.snapshot();
  EXPECT_FALSE(snapshot.cancellation_requested);
  EXPECT_FALSE(snapshot.pause_requested);
  EXPECT_FALSE(snapshot.paused_at_checkpoint);
  EXPECT_EQ(control.checkpoint(), CheckpointResult::kContinue);
}

TEST(ExecutionControl, ResumeReleasesPausedCheckpoint)
{
  ExecutionControl control;
  control.request_pause();

  auto checkpoint = std::async(
    std::launch::async, [&control]() {return control.checkpoint();});

  ASSERT_TRUE(wait_for_pause_acknowledgement(control, ExecutionControl::Clock::now() + 1s));
  EXPECT_TRUE(control.pause_requested());
  EXPECT_EQ(checkpoint.wait_for(0ms), std::future_status::timeout);

  control.resume();

  ASSERT_EQ(checkpoint.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(checkpoint.get(), CheckpointResult::kContinue);
  const auto snapshot = control.snapshot();
  EXPECT_FALSE(snapshot.pause_requested);
  EXPECT_FALSE(snapshot.paused_at_checkpoint);
}

TEST(ExecutionControl, CancelWakesPausedCheckpoint)
{
  ExecutionControl control;
  control.request_pause();

  auto checkpoint = std::async(
    std::launch::async, [&control]() {return control.checkpoint();});
  ASSERT_TRUE(wait_for_pause_acknowledgement(control, ExecutionControl::Clock::now() + 1s));

  control.request_cancel();

  ASSERT_EQ(checkpoint.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(checkpoint.get(), CheckpointResult::kCanceled);
  EXPECT_TRUE(control.cancellation_requested());
  EXPECT_EQ(control.checkpoint(), CheckpointResult::kCanceled);
}

TEST(ExecutionControl, CancelInterruptsLongSteadyClockWait)
{
  ExecutionControl control;
  std::promise<void> worker_started;
  auto started = worker_started.get_future();
  auto wait_result = std::async(
    std::launch::async,
    [&control, &worker_started]() {
      worker_started.set_value();
      return control.wait_for(std::chrono::hours(1));
    });
  started.get();

  control.request_cancel();

  ASSERT_EQ(wait_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(wait_result.get(), WaitResult::kCanceled);
}

TEST(ExecutionControl, ExpiredWaitCompletesWithoutSleeping)
{
  ExecutionControl control;

  EXPECT_EQ(control.wait_until(ExecutionControl::Clock::now()), WaitResult::kCompleted);
  EXPECT_EQ(control.wait_for(ExecutionControl::Duration::zero()), WaitResult::kCompleted);
}

TEST(ExecutionControl, ExistingCancellationTakesPriorityOverExpiredDeadline)
{
  ExecutionControl control;
  control.request_cancel();

  EXPECT_EQ(control.wait_until(ExecutionControl::Clock::now()), WaitResult::kCanceled);
  EXPECT_EQ(control.wait_for(-1ms), WaitResult::kCanceled);
}

}  // namespace
