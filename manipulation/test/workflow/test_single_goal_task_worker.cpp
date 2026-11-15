// Responsibility: Verifies worker ownership, cancellation, completion, and thread teardown.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "manipulation/workflow/single_goal_task_worker.hpp"

namespace manipulation::workflow
{
namespace
{

using namespace std::chrono_literals;
using manipulation::task::CheckpointResult;
using manipulation::task::ExecutionControl;
using manipulation::task::ObjectDisposition;
using manipulation::task::TaskErrorCode;
using manipulation::task::TaskOutcome;
using manipulation::task::WaitResult;

struct ThreadExitGateState
{
  std::promise<void> entered;
  std::promise<void> release;
  std::shared_future<void> release_future{release.get_future().share()};
};

class BlockThreadExit final
{
public:
  explicit BlockThreadExit(std::shared_ptr<ThreadExitGateState> state)
  : state_(std::move(state))
  {
  }

  ~BlockThreadExit()
  {
    state_->entered.set_value();
    state_->release_future.wait();
  }

private:
  std::shared_ptr<ThreadExitGateState> state_;
};

class ReleaseThreadExitOnScopeExit final
{
public:
  explicit ReleaseThreadExitOnScopeExit(std::shared_ptr<ThreadExitGateState> state)
  : state_(std::move(state))
  {
  }

  ~ReleaseThreadExitOnScopeExit()
  {
    try {
      state_->release.set_value();
    } catch (const std::future_error &) {
      // The normal path already released the gate.
    }
  }

private:
  std::shared_ptr<ThreadExitGateState> state_;
};

TaskOutcome success()
{
  return TaskOutcome::make_success(ObjectDisposition::kSafeIdle);
}

TaskOutcome canceled()
{
  return TaskOutcome::make_failure(
    TaskErrorCode::kCanceled, ObjectDisposition::kSafeIdle);
}

bool wait_for_state(
  const SingleGoalTaskWorker & worker, const TaskWorkerState expected,
  const std::chrono::steady_clock::duration timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (worker.snapshot().state == expected) {
      return true;
    }
    std::this_thread::yield();
  }
  return worker.snapshot().state == expected;
}

bool wait_until_paused(
  const SingleGoalTaskWorker & worker,
  const std::chrono::steady_clock::duration timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (worker.snapshot().execution_control.paused_at_checkpoint) {
      return true;
    }
    std::this_thread::yield();
  }
  return worker.snapshot().execution_control.paused_at_checkpoint;
}

TEST(SingleGoalTaskWorker, RejectsConcurrentSecondTask)
{
  SingleGoalTaskWorker worker;
  std::promise<void> entered;
  std::promise<TaskOutcome> completed;

  EXPECT_EQ(
    worker.start(
      [&entered](ExecutionControl & control) {
        entered.set_value();
        return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
      },
      [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
    TaskStartResult::kStarted);
  ASSERT_EQ(entered.get_future().wait_for(1s), std::future_status::ready);

  EXPECT_EQ(
    worker.start(
      [](ExecutionControl &) {return success();},
      [](TaskOutcome) {}),
    TaskStartResult::kBusy);
  EXPECT_TRUE(worker.request_cancel());

  auto result = completed.get_future();
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(result.get().error_code(), TaskErrorCode::kCanceled);
}

TEST(SingleGoalTaskWorker, AtomicallyAcceptsOnlyOneSimultaneousInitialStart)
{
  SingleGoalTaskWorker worker;
  std::promise<void> start_gate;
  const auto shared_start_gate = start_gate.get_future().share();
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();
  TaskStartResult first_result{TaskStartResult::kBusy};
  TaskStartResult second_result{TaskStartResult::kBusy};

  const auto task = [](ExecutionControl & control) {
      return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
    };
  const auto callback = [&completed](TaskOutcome outcome) {
      completed.set_value(std::move(outcome));
    };

  std::thread first_starter(
    [&] {
      shared_start_gate.wait();
      first_result = worker.start(task, callback);
    });
  std::thread second_starter(
    [&] {
      shared_start_gate.wait();
      second_result = worker.start(task, callback);
    });

  start_gate.set_value();
  first_starter.join();
  second_starter.join();

  const auto accepted_count =
    (first_result == TaskStartResult::kStarted ? 1U : 0U) +
    (second_result == TaskStartResult::kStarted ? 1U : 0U);
  EXPECT_EQ(accepted_count, 1U);
  EXPECT_TRUE(worker.request_cancel());
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(result.get().error_code(), TaskErrorCode::kCanceled);
}

TEST(SingleGoalTaskWorker, CancellationDuringStartingIsDeliveredToNewTask)
{
  SingleGoalTaskWorker worker;
  std::promise<void> finish_first_task;
  const auto first_task_gate = finish_first_task.get_future().share();
  const auto exit_gate_state = std::make_shared<ThreadExitGateState>();
  const ReleaseThreadExitOnScopeExit release_on_failure(exit_gate_state);
  auto exit_entered = exit_gate_state->entered.get_future();
  auto exit_blocker = std::make_shared<BlockThreadExit>(exit_gate_state);

  ASSERT_EQ(
    worker.start(
      [first_task_gate](ExecutionControl &) {
        first_task_gate.wait();
        return success();
      },
      [exit_blocker](TaskOutcome) {}),
    TaskStartResult::kStarted);

  // Leave the blocker owned only by the worker callback, then finish the task.
  exit_blocker.reset();
  finish_first_task.set_value();
  ASSERT_EQ(exit_entered.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));

  std::promise<TaskStartResult> second_start;
  auto second_start_result = second_start.get_future();
  std::promise<TaskOutcome> second_completed;
  auto second_outcome = second_completed.get_future();
  std::thread starter(
    [&] {
      second_start.set_value(
        worker.start(
          [](ExecutionControl & control) {
            return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
          },
          [&second_completed](TaskOutcome outcome) {
            second_completed.set_value(std::move(outcome));
          }));
    });

  EXPECT_TRUE(wait_for_state(worker, TaskWorkerState::kStarting));
  const auto starting = worker.snapshot();
  EXPECT_TRUE(starting.control_available);
  EXPECT_FALSE(starting.owned_thread_joinable);
  EXPECT_TRUE(worker.request_cancel());

  exit_gate_state->release.set_value();
  starter.join();
  ASSERT_EQ(second_start_result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(second_start_result.get(), TaskStartResult::kStarted);
  ASSERT_EQ(second_outcome.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(second_outcome.get().error_code(), TaskErrorCode::kCanceled);
}

TEST(SingleGoalTaskWorker, CancellationWakesInterruptibleWait)
{
  SingleGoalTaskWorker worker;
  std::promise<void> entered;
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();

  ASSERT_EQ(
    worker.start(
      [&entered](ExecutionControl & control) {
        entered.set_value();
        return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
      },
      [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
    TaskStartResult::kStarted);
  ASSERT_EQ(entered.get_future().wait_for(1s), std::future_status::ready);

  EXPECT_TRUE(worker.request_cancel());
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(result.get().error_code(), TaskErrorCode::kCanceled);
}

TEST(SingleGoalTaskWorker, PauseAndResumeOperateAtCheckpoint)
{
  SingleGoalTaskWorker worker;
  std::promise<void> ready;
  std::promise<void> enter_checkpoint;
  auto checkpoint_gate = enter_checkpoint.get_future().share();
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();

  ASSERT_EQ(
    worker.start(
      [&ready, checkpoint_gate](ExecutionControl & control) {
        ready.set_value();
        checkpoint_gate.wait();
        return control.checkpoint() == CheckpointResult::kCanceled ? canceled() : success();
      },
      [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
    TaskStartResult::kStarted);
  ASSERT_EQ(ready.get_future().wait_for(1s), std::future_status::ready);

  EXPECT_TRUE(worker.request_pause());
  enter_checkpoint.set_value();
  ASSERT_TRUE(wait_until_paused(worker));
  EXPECT_EQ(result.wait_for(0ms), std::future_status::timeout);

  EXPECT_TRUE(worker.resume());
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(result.get().succeeded());
}

TEST(SingleGoalTaskWorker, ConvertsTaskExceptionToTypedInternalError)
{
  SingleGoalTaskWorker worker;
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();
  std::atomic_uint32_t callback_count{0U};

  ASSERT_EQ(
    worker.start(
      [](ExecutionControl &) -> TaskOutcome {throw std::runtime_error("planner exploded");},
      [&completed, &callback_count](TaskOutcome outcome) {
        ++callback_count;
        completed.set_value(std::move(outcome));
      }),
    TaskStartResult::kStarted);
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);

  const auto outcome = result.get();
  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), TaskErrorCode::kInternalError);
  EXPECT_EQ(outcome.disposition(), ObjectDisposition::kUnknown);
  EXPECT_NE(outcome.detail().find("planner exploded"), std::string::npos);
  EXPECT_EQ(callback_count.load(), 1U);
}

TEST(SingleGoalTaskWorker, ConvertsUnknownTaskExceptionToTypedInternalError)
{
  SingleGoalTaskWorker worker;
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();

  ASSERT_EQ(
    worker.start(
      [](ExecutionControl &) -> TaskOutcome {throw 42;},
      [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
    TaskStartResult::kStarted);
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);

  const auto outcome = result.get();
  EXPECT_EQ(outcome.error_code(), TaskErrorCode::kInternalError);
  EXPECT_EQ(outcome.disposition(), ObjectDisposition::kUnknown);
  EXPECT_NE(outcome.detail().find("unknown exception"), std::string::npos);
}

TEST(SingleGoalTaskWorker, DestructorCancelsAndJoinsOwnedThread)
{
  std::promise<void> entered;
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();

  {
    SingleGoalTaskWorker worker;
    ASSERT_EQ(
      worker.start(
        [&entered](ExecutionControl & control) {
          entered.set_value();
          return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
        },
        [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
      TaskStartResult::kStarted);
    ASSERT_EQ(entered.get_future().wait_for(1s), std::future_status::ready);
  }

  ASSERT_EQ(result.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(result.get().error_code(), TaskErrorCode::kCanceled);
}

TEST(SingleGoalTaskWorker, ReapsCompletedThreadAndSupportsSequentialReuse)
{
  SingleGoalTaskWorker worker;
  std::promise<TaskOutcome> first_completed;
  auto first_result = first_completed.get_future();

  ASSERT_EQ(
    worker.start(
      [](ExecutionControl &) {return success();},
      [&first_completed](TaskOutcome outcome) {
        first_completed.set_value(std::move(outcome));
      }),
    TaskStartResult::kStarted);
  ASSERT_EQ(first_result.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));

  std::promise<TaskOutcome> second_completed;
  auto second_result = second_completed.get_future();
  EXPECT_EQ(
    worker.start(
      [](ExecutionControl &) {return success();},
      [&second_completed](TaskOutcome outcome) {
        second_completed.set_value(std::move(outcome));
      }),
    TaskStartResult::kStarted);
  ASSERT_EQ(second_result.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));

  const auto snapshot = worker.snapshot();
  EXPECT_EQ(snapshot.generation, 2U);
  EXPECT_TRUE(snapshot.owned_thread_joinable);
  EXPECT_TRUE(first_result.get().succeeded());
  EXPECT_TRUE(second_result.get().succeeded());
}

TEST(SingleGoalTaskWorker, StartFromCompletionCallbackIsRejectedWithoutSelfJoin)
{
  SingleGoalTaskWorker worker;
  std::promise<TaskStartResult> nested_start;
  auto nested_result = nested_start.get_future();

  ASSERT_EQ(
    worker.start(
      [](ExecutionControl &) {return success();},
      [&worker, &nested_start](TaskOutcome) {
        nested_start.set_value(
          worker.start(
            [](ExecutionControl &) {return success();},
            [](TaskOutcome) {}));
      }),
    TaskStartResult::kStarted);

  ASSERT_EQ(nested_result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(nested_result.get(), TaskStartResult::kBusy);
  EXPECT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));
}

TEST(SingleGoalTaskWorker, RecordsCompletionCallbackExceptionWithoutRetry)
{
  SingleGoalTaskWorker worker;
  std::atomic_uint32_t callback_count{0U};

  ASSERT_EQ(
    worker.start(
      [](ExecutionControl &) {return success();},
      [&callback_count](TaskOutcome) {
        ++callback_count;
        throw std::runtime_error("action completion failed");
      }),
    TaskStartResult::kStarted);
  ASSERT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));

  const auto snapshot = worker.snapshot();
  EXPECT_TRUE(snapshot.completion_callback_failed);
  EXPECT_EQ(callback_count.load(), 1U);
}

TEST(SingleGoalTaskWorker, SnapshotDescribesLifecycleAndControlState)
{
  SingleGoalTaskWorker worker;
  const auto idle = worker.snapshot();
  EXPECT_EQ(idle.state, TaskWorkerState::kIdle);
  EXPECT_FALSE(idle.owned_thread_joinable);
  EXPECT_FALSE(idle.control_available);
  EXPECT_EQ(idle.generation, 0U);

  std::promise<void> entered;
  std::promise<TaskOutcome> completed;
  auto result = completed.get_future();
  ASSERT_EQ(
    worker.start(
      [&entered](ExecutionControl & control) {
        entered.set_value();
        return control.wait_for(1h) == WaitResult::kCanceled ? canceled() : success();
      },
      [&completed](TaskOutcome outcome) {completed.set_value(std::move(outcome));}),
    TaskStartResult::kStarted);
  ASSERT_EQ(entered.get_future().wait_for(1s), std::future_status::ready);

  const auto running = worker.snapshot();
  EXPECT_EQ(running.state, TaskWorkerState::kRunning);
  EXPECT_TRUE(running.owned_thread_joinable);
  EXPECT_TRUE(running.control_available);
  EXPECT_EQ(running.generation, 1U);

  EXPECT_TRUE(worker.request_cancel());
  ASSERT_EQ(result.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(wait_for_state(worker, TaskWorkerState::kCompleted));
  const auto completed_snapshot = worker.snapshot();
  EXPECT_TRUE(completed_snapshot.owned_thread_joinable);
  EXPECT_TRUE(completed_snapshot.execution_control.cancellation_requested);
}

TEST(SingleGoalTaskWorker, RejectsEmptyCallables)
{
  SingleGoalTaskWorker worker;
  EXPECT_THROW(
    worker.start({}, [](TaskOutcome) {}),
    std::invalid_argument);
  EXPECT_THROW(
    worker.start([](ExecutionControl &) {return success();}, {}),
    std::invalid_argument);
}

}  // namespace
}  // namespace manipulation::workflow
