// Responsibility: Verifies operation-gate ordering against concurrent pause and completion.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "manipulation/workflow/detail/transfer_sequence_session.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using manipulation::task::ObjectDisposition;
using manipulation::task::TaskOutcome;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

TaskOutcome success()
{
  return TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget);
}

TEST(TransferSequenceSessionOperationRace, PauseWaitsForTheAdmittedOperation)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  std::promise<void> operation_entered;
  std::promise<void> release_operation;
  auto entered = operation_entered.get_future();
  const auto release = release_operation.get_future().share();
  TaskOperationGateResult operation_result{TaskOperationGateResult::kStopped};

  std::thread operation_thread(
    [&]() {
      operation_result = session.run_operation(
        std::chrono::steady_clock::now() + 10s,
        [&]() {
          operation_entered.set_value();
          release.wait();
        });
    });
  if (entered.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    FAIL() << "operation did not enter the session gate";
  }

  std::promise<void> pause_attempting;
  auto attempting = pause_attempting.get_future();
  auto pause_future = std::async(
    std::launch::async,
    [&]() {
      pause_attempting.set_value();
      return session.set_pause_requested(true);
    });
  if (attempting.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    (void)pause_future.wait_for(1s);
    FAIL() << "pause request did not reach the session gate";
  }

  EXPECT_EQ(pause_future.wait_for(100ms), std::future_status::timeout);
  release_operation.set_value();
  operation_thread.join();
  ASSERT_EQ(pause_future.wait_for(1s), std::future_status::ready);

  EXPECT_EQ(operation_result, TaskOperationGateResult::kExecuted);
  EXPECT_TRUE(pause_future.get());
  EXPECT_TRUE(session.pause_requested());
}

TEST(TransferSequenceSessionOperationRace, PauseBeforeDispatchRejectsTheOperation)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  ASSERT_TRUE(session.set_pause_requested(true));
  int dispatch_count = 0;

  const auto result = session.run_operation(
    std::chrono::steady_clock::now() + 10s,
    [&dispatch_count]() {++dispatch_count;});

  EXPECT_EQ(result, TaskOperationGateResult::kPaused);
  EXPECT_EQ(dispatch_count, 0);
}

TEST(TransferSequenceSessionOperationRace, CancelBeforeDispatchRejectsTheOperation)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  ASSERT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  int dispatch_count = 0;

  const auto result = session.run_operation(
    std::chrono::steady_clock::now() + 10s,
    [&dispatch_count]() {++dispatch_count;});

  EXPECT_EQ(result, TaskOperationGateResult::kStopped);
  EXPECT_EQ(dispatch_count, 0);
}

TEST(TransferSequenceSessionOperationRace, MaintenanceIgnoresPauseButNeverPassesStop)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  ASSERT_TRUE(session.set_pause_requested(true));
  int maintenance_count = 0;

  EXPECT_EQ(
    session.run_maintenance_operation(
      std::chrono::steady_clock::now() + 10s,
      [&maintenance_count]() {++maintenance_count;}),
    TaskOperationGateResult::kExecuted);
  EXPECT_EQ(maintenance_count, 1);

  ASSERT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_EQ(
    session.run_maintenance_operation(
      std::chrono::steady_clock::now() + 10s,
      [&maintenance_count]() {++maintenance_count;}),
    TaskOperationGateResult::kStopped);
  EXPECT_EQ(maintenance_count, 1);
}

TEST(TransferSequenceSessionOperationRace, CancelWaitsForAdmittedDispatchAndBlocksTheNextOne)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  std::promise<void> operation_entered;
  std::promise<void> release_operation;
  auto entered = operation_entered.get_future();
  const auto release = release_operation.get_future().share();
  TaskOperationGateResult operation_result{TaskOperationGateResult::kStopped};

  std::thread operation_thread(
    [&]() {
      operation_result = session.run_operation(
        std::chrono::steady_clock::now() + 10s,
        [&]() {
          operation_entered.set_value();
          release.wait();
        });
    });
  if (entered.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    FAIL() << "operation did not enter the session gate";
  }

  std::promise<void> cancel_attempting;
  auto attempting = cancel_attempting.get_future();
  auto cancel_future = std::async(
    std::launch::async,
    [&]() {
      cancel_attempting.set_value();
      return session.request_stop(TaskStopCause::kClientCanceled);
    });
  if (attempting.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    (void)cancel_future.wait_for(1s);
    FAIL() << "cancel request did not reach the session gate";
  }
  EXPECT_EQ(cancel_future.wait_for(100ms), std::future_status::timeout);

  release_operation.set_value();
  operation_thread.join();
  ASSERT_EQ(cancel_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(operation_result, TaskOperationGateResult::kExecuted);
  EXPECT_TRUE(cancel_future.get());

  int later_dispatch_count = 0;
  EXPECT_EQ(
    session.run_operation(
      std::chrono::steady_clock::now() + 10s,
      [&later_dispatch_count]() {++later_dispatch_count;}),
    TaskOperationGateResult::kStopped);
  EXPECT_EQ(later_dispatch_count, 0);
}

TEST(TransferSequenceSessionOperationRace, CompletionWaitsForTheAdmittedOperation)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  std::promise<void> operation_entered;
  std::promise<void> release_operation;
  auto entered = operation_entered.get_future();
  const auto release = release_operation.get_future().share();
  TaskOperationGateResult operation_result{TaskOperationGateResult::kStopped};

  std::thread operation_thread(
    [&]() {
      operation_result = session.run_operation(
        std::chrono::steady_clock::now() + 10s,
        [&]() {
          operation_entered.set_value();
          release.wait();
        });
    });
  if (entered.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    FAIL() << "operation did not enter the session gate";
  }

  std::promise<void> completion_attempting;
  auto attempting = completion_attempting.get_future();
  auto completion_future = std::async(
    std::launch::async,
    [&]() {
      completion_attempting.set_value();
      return session.queue_completion(success());
    });
  if (attempting.wait_for(1s) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    (void)completion_future.wait_for(1s);
    FAIL() << "completion did not reach the session gate";
  }

  EXPECT_EQ(completion_future.wait_for(100ms), std::future_status::timeout);
  release_operation.set_value();
  operation_thread.join();
  ASSERT_EQ(completion_future.wait_for(1s), std::future_status::ready);

  EXPECT_EQ(operation_result, TaskOperationGateResult::kExecuted);
  EXPECT_TRUE(completion_future.get());
  const auto snapshot = session.snapshot();
  EXPECT_EQ(snapshot.phase, TransferSequenceSessionPhase::kTerminalReady);
  EXPECT_TRUE(snapshot.completion_ready);
}

}  // namespace
}  // namespace manipulation::workflow::detail
