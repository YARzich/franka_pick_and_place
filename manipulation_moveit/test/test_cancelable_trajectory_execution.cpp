// Responsibility: Verify exact-goal cancellation, serialized dispatch, and confirmed stop.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cancelable_trajectory_execution.hpp"

namespace manipulation::moveit::detail
{
namespace
{

using namespace std::chrono_literals;

TrajectoryGoalId goal_id(uint8_t value)
{
  TrajectoryGoalId result{};
  result[0] = value;
  return result;
}

class Gate
{
public:
  void close()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
  }

  void release()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    cv_.notify_all();
  }

  void wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, 3s, [this]() {return open_;})) {
      throw std::runtime_error("test gate was not released");
    }
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_{true};
};

class ExecutionHarness
{
public:
  ~ExecutionHarness()
  {
    push_gate.release();
    stop_gate.release();
    join_gate.release();
    finish_gate.release();
    execution.shutdown();
  }

  TrajectoryExecutionOperations operations()
  {
    TrajectoryExecutionOperations result;
    result.push = [this]() {
        record("push");
        push_gate.wait();
        return push_result;
      };
    result.start = [this](std::function<void(TrajectoryExecutionStatus)> completion) {
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          completion_ = completion;
        }
        record("start");
        if (start_throws) {
          throw std::runtime_error("start failed after dispatch");
        }
        if (inline_completion) {
          completion(*inline_completion);
        }
      };
    result.stop = [this]() {
        record("stop");
        stop_gate.wait();
        if (stop_throws) {
          throw std::runtime_error("physical stop is unconfirmed");
        }
        record("stopped");
      };
    result.join = [this]() {
        record("join");
        join_gate.wait();
        record("joined");
      };
    result.finish = [this](const TrajectoryExecutionOutcome & outcome) {
        record("finish");
        finish_gate.wait();
        const std::lock_guard<std::mutex> lock(mutex_);
        outcomes_.push_back(outcome);
        cv_.notify_all();
      };
    result.fault = [this](const std::string & detail) {
        const std::lock_guard<std::mutex> lock(mutex_);
        faults_.push_back(detail);
        cv_.notify_all();
      };
    return result;
  }

  bool wait_for_event(const std::string & event)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [&]() {
               return std::find(events_.begin(), events_.end(), event) != events_.end();
      });
  }

  bool wait_for_outcome()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [this]() {return !outcomes_.empty();});
  }

  bool wait_for_fault()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [this]() {return !faults_.empty();});
  }

  void complete(TrajectoryExecutionStatus status)
  {
    std::function<void(TrajectoryExecutionStatus)> completion;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      completion = completion_;
    }
    ASSERT_TRUE(static_cast<bool>(completion));
    completion(status);
  }

  std::vector<std::string> events()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  std::vector<TrajectoryExecutionOutcome> outcomes()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return outcomes_;
  }

  std::vector<std::thread::id> operation_threads()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return threads_;
  }

  bool push_result{true};
  bool start_throws{false};
  bool stop_throws{false};
  std::optional<TrajectoryExecutionStatus> inline_completion;
  Gate push_gate;
  Gate stop_gate;
  Gate join_gate;
  Gate finish_gate;
  CancelableTrajectoryExecution execution;

private:
  void record(const std::string & event)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(event);
    threads_.push_back(std::this_thread::get_id());
    cv_.notify_all();
  }

  std::mutex mutex_;
  std::condition_variable cv_;
  std::function<void(TrajectoryExecutionStatus)> completion_;
  std::vector<std::string> events_;
  std::vector<std::thread::id> threads_;
  std::vector<TrajectoryExecutionOutcome> outcomes_;
  std::vector<std::string> faults_;
};

TEST(CancelableTrajectoryExecutionTest, CancelBeforeSubmissionNeverDispatchesPhysicalOperations)
{
  ExecutionHarness harness;
  const auto id = goal_id(1);
  ASSERT_TRUE(harness.execution.reserve(id));
  ASSERT_TRUE(harness.execution.request_cancel(id));
  ASSERT_TRUE(harness.execution.submit(id, harness.operations()));

  ASSERT_TRUE(harness.wait_for_outcome());
  EXPECT_EQ(harness.events(), std::vector<std::string>({"finish"}));
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kCanceled);
}

TEST(CancelableTrajectoryExecutionTest, NaturalCompletionJoinsBeforeReportingSuccess)
{
  ExecutionHarness harness;
  harness.join_gate.close();
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("start"));
  EXPECT_TRUE(harness.outcomes().empty());

  harness.complete(TrajectoryExecutionStatus::kSucceeded);
  ASSERT_TRUE(harness.wait_for_event("join"));
  EXPECT_TRUE(harness.outcomes().empty());
  harness.join_gate.release();
  ASSERT_TRUE(harness.wait_for_outcome());

  EXPECT_EQ(harness.events(),
        std::vector<std::string>({"push", "start", "join", "joined", "finish"}));
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kSucceeded);
  const auto threads = harness.operation_threads();
  ASSERT_FALSE(threads.empty());
  EXPECT_NE(threads.front(), std::this_thread::get_id());
  EXPECT_TRUE(std::all_of(threads.begin(), threads.end(), [&](const auto & id) {
      return id == threads.front();
    }));
}

TEST(CancelableTrajectoryExecutionTest, ActiveCancelReportsTerminalOnlyAfterConfirmedStop)
{
  ExecutionHarness harness;
  harness.stop_gate.close();
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("start"));

  ASSERT_TRUE(harness.execution.request_cancel(goal_id(1)));
  ASSERT_TRUE(harness.wait_for_event("stop"));
  EXPECT_TRUE(harness.outcomes().empty());
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  harness.stop_gate.release();
  ASSERT_TRUE(harness.wait_for_outcome());

  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kCanceled);
  const auto events = harness.events();
  EXPECT_EQ(std::count(events.begin(), events.end(), "stop"), 1);
  EXPECT_LT(std::find(events.begin(), events.end(), "stopped"),
    std::find(events.begin(), events.end(), "finish"));
}

TEST(CancelableTrajectoryExecutionTest, CancelDuringPushCompletesDispatchBeforeStopping)
{
  ExecutionHarness harness;
  harness.push_gate.close();
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("push"));

  ASSERT_TRUE(harness.execution.request_cancel(goal_id(1)));
  EXPECT_TRUE(harness.outcomes().empty());
  harness.push_gate.release();
  ASSERT_TRUE(harness.wait_for_outcome());

  const auto events = harness.events();
  EXPECT_LT(std::find(events.begin(), events.end(), "start"),
    std::find(events.begin(), events.end(), "stop"));
  EXPECT_EQ(std::count(events.begin(), events.end(), "start"), 1);
  EXPECT_EQ(std::count(events.begin(), events.end(), "stop"), 1);
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kCanceled);
}

TEST(CancelableTrajectoryExecutionTest, WrongGoalCannotCancelOrReplaceActiveExecution)
{
  ExecutionHarness harness;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  EXPECT_FALSE(harness.execution.submit(goal_id(2), harness.operations()));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("start"));

  EXPECT_FALSE(harness.execution.request_cancel(goal_id(2)));
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  harness.complete(TrajectoryExecutionStatus::kSucceeded);
  ASSERT_TRUE(harness.wait_for_outcome());
  const auto events = harness.events();
  EXPECT_EQ(std::count(events.begin(), events.end(), "stop"), 0);
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kSucceeded);
}

TEST(CancelableTrajectoryExecutionTest, FinishingGoalRetainsExclusiveAdmission)
{
  ExecutionHarness harness;
  harness.finish_gate.close();
  harness.inline_completion = TrajectoryExecutionStatus::kSucceeded;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("finish"));

  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  EXPECT_FALSE(harness.execution.submit(goal_id(2), harness.operations()));
  harness.finish_gate.release();
  ASSERT_TRUE(harness.wait_for_outcome());
}

TEST(CancelableTrajectoryExecutionTest, InlineFailedCompletionDoesNotDeadlock)
{
  ExecutionHarness harness;
  harness.inline_completion = TrajectoryExecutionStatus::kFailed;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));

  ASSERT_TRUE(harness.wait_for_outcome());
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kFailed);
}

TEST(CancelableTrajectoryExecutionTest, RejectedPushNeverStartsExecution)
{
  ExecutionHarness harness;
  harness.push_result = false;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));

  ASSERT_TRUE(harness.wait_for_outcome());
  EXPECT_EQ(harness.events(), std::vector<std::string>({"push", "finish"}));
  EXPECT_EQ(harness.outcomes().front().status, TrajectoryExecutionStatus::kFailed);
}

TEST(CancelableTrajectoryExecutionTest, ShutdownWaitsForPhysicalStopAndJoinsWorker)
{
  ExecutionHarness harness;
  harness.stop_gate.close();
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("start"));

  auto shutdown = std::async(std::launch::async, [&]() {harness.execution.shutdown();});
  const bool stop_started = harness.wait_for_event("stop");
  EXPECT_TRUE(stop_started);
  EXPECT_EQ(shutdown.wait_for(20ms), std::future_status::timeout);
  harness.stop_gate.release();
  ASSERT_EQ(shutdown.wait_for(1s), std::future_status::ready);
  shutdown.get();

  const auto events = harness.events();
  EXPECT_EQ(std::count(events.begin(), events.end(), "stopped"), 1);
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  harness.execution.shutdown();
}

TEST(CancelableTrajectoryExecutionTest, StartExceptionClosesAdmissionInsteadOfClaimingSuccess)
{
  ExecutionHarness harness;
  harness.start_throws = true;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));

  ASSERT_TRUE(harness.wait_for_fault());
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  EXPECT_TRUE(harness.outcomes().empty());
}

TEST(CancelableTrajectoryExecutionTest, StopExceptionRetainsUncertaintyWithoutTerminalResult)
{
  ExecutionHarness harness;
  harness.stop_throws = true;
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), harness.operations()));
  ASSERT_TRUE(harness.wait_for_event("start"));
  ASSERT_TRUE(harness.execution.request_cancel(goal_id(1)));

  ASSERT_TRUE(harness.wait_for_fault());
  EXPECT_TRUE(harness.outcomes().empty());
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
  EXPECT_FALSE(harness.execution.submit(goal_id(2), harness.operations()));
}

TEST(CancelableTrajectoryExecutionTest, FaultRetainsGoalOwnershipUntilHelperShutdown)
{
  struct GoalLifetime
  {
    std::promise<void> released;
    ~GoalLifetime() {released.set_value();}
  };

  ExecutionHarness harness;
  harness.stop_throws = true;
  auto lifetime = std::make_shared<GoalLifetime>();
  const std::weak_ptr<GoalLifetime> retained_goal = lifetime;
  auto released = lifetime->released.get_future();
  auto operations = harness.operations();
  operations.finish = [
    owned_goal = lifetime, finish = std::move(operations.finish)
    ](const TrajectoryExecutionOutcome & outcome) {
      finish(outcome);
    };
  ASSERT_TRUE(harness.execution.reserve(goal_id(1)));
  ASSERT_TRUE(harness.execution.submit(goal_id(1), std::move(operations)));
  operations = {};
  lifetime.reset();
  ASSERT_TRUE(harness.wait_for_event("start"));
  ASSERT_TRUE(harness.execution.request_cancel(goal_id(1)));
  ASSERT_TRUE(harness.wait_for_fault());

  // Losing the last real ServerGoalHandle owner can itself publish a terminal
  // result. Keeping finish() unused alone does not prove safe fault retention.
  EXPECT_EQ(released.wait_for(100ms), std::future_status::timeout);
  EXPECT_FALSE(retained_goal.expired());
  EXPECT_TRUE(harness.outcomes().empty());
  EXPECT_FALSE(harness.execution.reserve(goal_id(2)));
}

}  // namespace
}  // namespace manipulation::moveit::detail
