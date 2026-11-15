// Responsibility: Exercise exact-goal planning cancel before context creation and worker exit.

#include <atomic>
#include <chrono>
#include <future>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"
#include "goal_planning_cancellation.hpp"

namespace manipulation::moveit::detail
{
namespace
{
using namespace std::chrono_literals;
const GoalPlanningCancellation::GoalId kGoal{1};
const GoalPlanningCancellation::GoalId kOtherGoal{2};

TEST(GoalPlanningCancellation, CancelDuringRequestAdapterReachesLaterPlanningContext)
{
  GoalPlanningCancellation cancellation{kGoal};
  std::promise<void> adapter_entered;
  std::promise<void> leave_adapter;
  auto adapter_gate = leave_adapter.get_future();
  std::atomic_bool context_exists{false};
  std::atomic_bool context_stopped{false};
  std::atomic_bool release_for_cleanup{false};
  auto worker = std::async(std::launch::async, [&]() {
        return cancellation.run([&]() {
                 adapter_entered.set_value();
                 adapter_gate.wait();
                 context_exists.store(true);
                 while (!context_stopped.load() && !release_for_cleanup.load()) {
                   std::this_thread::sleep_for(1ms);
                 }
        }, [&]() {
                 if (context_exists.load()) {
                   context_stopped.store(true);
                 }
        });
    });
  ASSERT_EQ(adapter_entered.get_future().wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(cancellation.request(kGoal));
  EXPECT_EQ(worker.wait_for(0ms), std::future_status::timeout);
  leave_adapter.set_value();
  const auto finished = worker.wait_for(500ms);
  release_for_cleanup.store(true);
  worker.get();
  EXPECT_EQ(finished, std::future_status::ready);
  EXPECT_TRUE(context_stopped.load());
  EXPECT_TRUE(cancellation.commit_terminal());
}

TEST(GoalPlanningCancellation, PreDispatchCancelSkipsPlanningAndWrongGoalCannotCancel)
{
  GoalPlanningCancellation cancellation{kGoal};
  EXPECT_FALSE(cancellation.request(kOtherGoal));
  EXPECT_FALSE(cancellation.requested());
  EXPECT_TRUE(cancellation.request(kGoal));
  bool planned = false;
  EXPECT_FALSE(cancellation.run([&]() {planned = true;}, []() {}));
  EXPECT_FALSE(planned);
}

TEST(GoalPlanningCancellation, TerminalCommitRejectsLateCancelWithoutAffectingNextGoal)
{
  GoalPlanningCancellation old_goal{kGoal};
  EXPECT_TRUE(old_goal.run([]() {}, []() {}));
  EXPECT_FALSE(old_goal.commit_terminal());
  GoalPlanningCancellation next_goal{kOtherGoal};
  EXPECT_FALSE(old_goal.request(kGoal));
  EXPECT_FALSE(next_goal.request(kGoal));
  bool planned = false;
  EXPECT_TRUE(next_goal.run([&]() {planned = true;}, []() {FAIL();}));
  EXPECT_TRUE(planned);
  EXPECT_FALSE(next_goal.commit_terminal());
}

TEST(GoalPlanningCancellation, PlanningExceptionLeavesNoCancellationWorker)
{
  GoalPlanningCancellation cancellation{kGoal};
  EXPECT_THROW(cancellation.run([]() {throw std::runtime_error("planning");}, []() {}),
    std::runtime_error);
  EXPECT_FALSE(cancellation.commit_terminal());
  EXPECT_FALSE(cancellation.request(kGoal));
}

TEST(GoalPlanningCancellation, CancellationAtPlanningReturnIsKeptForTerminalSelection)
{
  GoalPlanningCancellation cancellation{kGoal};
  EXPECT_TRUE(cancellation.run([&]() {EXPECT_TRUE(cancellation.request(kGoal));}, []() {}));
  EXPECT_TRUE(cancellation.commit_terminal());
  EXPECT_FALSE(cancellation.request(kGoal));
}

TEST(GoalPlanningCancellation, TerminateExceptionDoesNotEscapeCancelOrDetachWatcher)
{
  GoalPlanningCancellation cancellation{kGoal};
  std::promise<void> planning;
  std::atomic_bool attempted{false};
  std::atomic_bool cleanup{false};
  auto worker = std::async(std::launch::async, [&]() {
        return cancellation.run([&]() {
                 planning.set_value();
                 while (!attempted.load() && !cleanup.load()) {
                   std::this_thread::sleep_for(1ms);
                 }
        }, [&]() {
                 attempted.store(true);
                 throw std::runtime_error("terminate failed");
        });
    });
  ASSERT_EQ(planning.get_future().wait_for(1s), std::future_status::ready);
  EXPECT_NO_THROW(EXPECT_TRUE(cancellation.request(kGoal)));
  const auto finished = worker.wait_for(500ms);
  cleanup.store(true);
  EXPECT_EQ(finished, std::future_status::ready);
  EXPECT_THROW(worker.get(), std::runtime_error);
  EXPECT_TRUE(cancellation.commit_terminal());
}

TEST(GoalPlanningCancellation, PlanningWorkerCannotReturnBeforeTerminationCallback)
{
  GoalPlanningCancellation cancellation{kGoal};
  std::promise<void> planning;
  std::promise<void> terminating;
  std::promise<void> leave_termination;
  auto gate = leave_termination.get_future();
  std::atomic_bool plan_may_return{false};
  auto worker = std::async(std::launch::async, [&]() {
        return cancellation.run([&]() {
                 planning.set_value();
                 while (!plan_may_return.load()) {
                   std::this_thread::sleep_for(1ms);
                 }
        }, [&]() {
                 terminating.set_value();
                 plan_may_return.store(true);
                 gate.wait();
        });
    });
  ASSERT_EQ(planning.get_future().wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(cancellation.request(kGoal));
  const auto entered = terminating.get_future().wait_for(1s);
  EXPECT_EQ(entered, std::future_status::ready);
  EXPECT_EQ(worker.wait_for(20ms), std::future_status::timeout);
  plan_may_return.store(true);
  leave_termination.set_value();
  EXPECT_TRUE(worker.get());
}

}  // namespace
}  // namespace manipulation::moveit::detail
