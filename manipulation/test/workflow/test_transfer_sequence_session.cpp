// Responsibility: Verifies goal-session stop-cause arbitration and backend invocation ownership.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <utility>

#include "manipulation/workflow/detail/transfer_sequence_session.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using manipulation::task::ObjectDisposition;
using manipulation::task::TaskOutcome;
using std::chrono_literals::operator""ms;

TaskOutcome success()
{
  return TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget);
}

TEST(TransferSequenceSession, StickyCancelPreventsBackendInvocationBeforeStart)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_TRUE(session.mark_worker_starting());
  EXPECT_FALSE(session.mark_backend_invoked());
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kClientCanceled);
  EXPECT_FALSE(session.snapshot().backend_invoked);
}

TEST(TransferSequenceSession, FirstStopCauseWinsAndLateCancelIsRejected)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.request_stop(TaskStopCause::kDeadlineExceeded));
  EXPECT_FALSE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_TRUE(session.request_stop(TaskStopCause::kDeadlineExceeded));
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kDeadlineExceeded);
}

TEST(TransferSequenceSession, CompletionFirstRejectsCancel)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.mark_worker_starting());
  EXPECT_TRUE(session.mark_worker_running());
  EXPECT_TRUE(session.mark_backend_invoked());
  EXPECT_TRUE(session.queue_completion(success()));
  EXPECT_FALSE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kNone);
}

TEST(TransferSequenceSession, CancelFirstNormalizesLaterCompletion)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.mark_worker_starting());
  EXPECT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_TRUE(session.queue_completion(success()));
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kClientCanceled);
  EXPECT_TRUE(session.snapshot().completion_ready);
}

TEST(TransferSequenceSession, CancelAndCompletionRaceHasOneValidLinearization)
{
  for (int iteration = 0; iteration < 200; ++iteration) {
    TransferSequenceSession session;
    ASSERT_TRUE(session.mark_worker_starting());
    ASSERT_TRUE(session.mark_worker_running());
    std::promise<void> gate;
    const auto shared_gate = gate.get_future().share();
    std::atomic_bool cancel_accepted{false};
    std::atomic_bool completion_queued{false};

    std::thread cancel_thread(
      [&] {
        shared_gate.wait();
        cancel_accepted.store(
          session.request_stop(TaskStopCause::kClientCanceled));
      });
    std::thread completion_thread(
      [&] {
        shared_gate.wait();
        completion_queued.store(session.queue_completion(success()));
      });
    gate.set_value();
    cancel_thread.join();
    completion_thread.join();

    EXPECT_TRUE(completion_queued.load());
    if (cancel_accepted.load()) {
      EXPECT_EQ(session.stop_cause(), TaskStopCause::kClientCanceled);
    } else {
      EXPECT_EQ(session.stop_cause(), TaskStopCause::kNone);
    }
  }
}

TEST(TransferSequenceSession, PauseCanBeStoredResumedAndCanceledBeforeWorker)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.set_pause_requested(true));
  EXPECT_TRUE(session.pause_requested());
  EXPECT_TRUE(session.set_pause_requested(false));
  EXPECT_FALSE(session.pause_requested());
  EXPECT_TRUE(session.set_pause_requested(true));
  EXPECT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kClientCanceled);
}

TEST(TransferSequenceSession, StopDispatchIsClaimedExactlyOnceAndAllowsReentry)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.request_stop(TaskStopCause::kDeadlineExceeded));
  int invocation_count = 0;
  EXPECT_TRUE(
    session.dispatch_stop(
      [&](const TaskStopCause cause) {
        ++invocation_count;
        EXPECT_EQ(cause, TaskStopCause::kDeadlineExceeded);
        // Re-entry proves the callback is not invoked under session mutex.
        EXPECT_EQ(
          session.snapshot().stop_cause,
          TaskStopCause::kDeadlineExceeded);
      }));
  EXPECT_EQ(invocation_count, 1);
  EXPECT_FALSE(session.dispatch_stop([&](auto) {++invocation_count;}));
  EXPECT_EQ(invocation_count, 1);
}

TEST(TransferSequenceSession, DeliveryCanBeDeferredAndOccursExactlyOnce)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.queue_completion(success()));
  auto first = session.take_completion();
  ASSERT_TRUE(first);
  EXPECT_FALSE(session.take_completion());

  session.restore_completion(std::move(*first));
  auto retried = session.take_completion();
  ASSERT_TRUE(retried);
  EXPECT_TRUE(session.mark_delivered());
  EXPECT_FALSE(session.mark_delivered());
  EXPECT_FALSE(session.take_completion());
  EXPECT_FALSE(session.queue_completion(success()));
}

TEST(TransferSequenceSession, ShutdownIsAStickyFirstCause)
{
  TransferSequenceSession session;
  EXPECT_TRUE(session.request_stop(TaskStopCause::kShutdown));
  EXPECT_FALSE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kShutdown);
}

TEST(TransferSequenceSession, OperationGateLinearizesPauseStopAndExecution)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  int operation_count = 0;
  const auto future_deadline = std::chrono::steady_clock::now() + 100ms;

  ASSERT_TRUE(session.set_pause_requested(true));
  EXPECT_EQ(
    session.run_operation(future_deadline, [&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kPaused);
  EXPECT_EQ(operation_count, 0);

  ASSERT_TRUE(session.set_pause_requested(false));
  EXPECT_EQ(
    session.run_operation(future_deadline, [&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kExecuted);
  EXPECT_EQ(operation_count, 1);

  ASSERT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
  EXPECT_EQ(
    session.run_operation(future_deadline, [&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kStopped);
  EXPECT_EQ(operation_count, 1);
}

TEST(TransferSequenceSession, OperationGateRecordsExactSteadyDeadlineUnderLock)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  int operation_count = 0;

  EXPECT_EQ(
    session.run_operation(
      std::chrono::steady_clock::now(), [&operation_count]() {++operation_count;}),
    TaskOperationGateResult::kStopped);
  EXPECT_EQ(operation_count, 0);
  EXPECT_EQ(session.stop_cause(), TaskStopCause::kDeadlineExceeded);
}

TEST(TransferSequenceSession, OperationAndCancelHaveOneDispatchLinearization)
{
  TransferSequenceSession session;
  ASSERT_TRUE(session.mark_worker_starting());
  ASSERT_TRUE(session.mark_worker_running());
  ASSERT_TRUE(session.mark_backend_invoked());
  std::promise<void> operation_entered;
  std::promise<void> release_operation;
  auto entered = operation_entered.get_future();
  const auto release = release_operation.get_future().share();
  std::atomic_bool cancel_attempting{false};
  std::atomic_bool cancel_returned{false};

  std::thread operation_thread(
    [&]() {
      EXPECT_EQ(
        session.run_operation(
          std::chrono::steady_clock::now() + 1000ms,
          [&]() {
            operation_entered.set_value();
            release.wait();
          }),
        TaskOperationGateResult::kExecuted);
    });
  if (entered.wait_for(1000ms) != std::future_status::ready) {
    release_operation.set_value();
    operation_thread.join();
    FAIL() << "operation did not enter the session gate";
  }
  std::thread cancel_thread(
    [&]() {
      cancel_attempting.store(true);
      EXPECT_TRUE(session.request_stop(TaskStopCause::kClientCanceled));
      cancel_returned.store(true);
    });

  const auto attempting_deadline = std::chrono::steady_clock::now() + 100ms;
  while (!cancel_attempting.load() &&
    std::chrono::steady_clock::now() < attempting_deadline)
  {
    std::this_thread::yield();
  }
  EXPECT_TRUE(cancel_attempting.load());
  EXPECT_FALSE(cancel_returned.load());
  release_operation.set_value();
  operation_thread.join();
  cancel_thread.join();
  EXPECT_TRUE(cancel_returned.load());
}

}  // namespace
}  // namespace manipulation::workflow::detail
