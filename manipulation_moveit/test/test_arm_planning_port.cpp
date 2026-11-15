// Responsibility: Verify bounded retry, stop, and scene-revision behavior without ROS runtime.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <mutex>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "manipulation_moveit/arm_planning_port.hpp"

#include "trajectory_state_validation.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;

class CountingExecutionContext final
  : public manipulation::workflow::TransferExecutionContext
{
public:
  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    checkpoints.fetch_add(1U);
    return manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return false;
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return checkpoint();
  }

  manipulation::workflow::TransferExecutionControlResult heartbeat() override
  {
    heartbeats.fetch_add(1U);
    return manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  std::atomic_size_t checkpoints{0U};
  std::atomic_size_t heartbeats{0U};
};

PlanningSceneRevision revision(const std::uint64_t value = 7U)
{
  return {"scene-session", value};
}

ArmPlanningRequest request_for(const PlanningSceneRevision & scene_revision = revision())
{
  ArmPlanningRequest request;
  request.goal_kind = ArmPlanningGoalKind::kJoint;
  request.joint_names = {"joint_1"};
  request.joint_positions = {0.25};
  request.label = "test";
  request.scene_revision = scene_revision;
  request.scene_snapshot.is_diff = false;
  request.scene_snapshot.robot_state.is_diff = false;
  request.scene_snapshot.robot_state.joint_state.name = {"joint_1"};
  request.scene_snapshot.robot_state.joint_state.position = {0.0};
  return request;
}

ArmPlanningResult success()
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_1"};
  trajectory.joint_trajectory.points.emplace_back();
  trajectory.joint_trajectory.points.front().positions = {0.0};
  return ArmPlanningResult::success(std::move(trajectory), "planned");
}

class ScriptedBackend final : public IArmPlanningBackend
{
public:
  explicit ScriptedBackend(std::vector<ArmPlanningResult> results)
  : results_(std::move(results))
  {
  }

  ArmPlanningReadiness cached_readiness() const override
  {
    return {true, "ready"};
  }

  ArmPlanningResult plan_once(
    const ArmPlanningRequest &,
    const PlanningStopToken &) override
  {
    const auto index = calls_.fetch_add(1U);
    if (index >= results_.size()) {
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, "script exhausted");
    }
    return results_[index];
  }

  void request_stop() noexcept override
  {
    stop_calls_.fetch_add(1U);
  }

  [[nodiscard]] std::size_t calls() const noexcept
  {
    return calls_.load();
  }

  [[nodiscard]] std::size_t stop_calls() const noexcept
  {
    return stop_calls_.load();
  }

private:
  std::vector<ArmPlanningResult> results_;
  std::atomic<std::size_t> calls_{0U};
  std::atomic<std::size_t> stop_calls_{0U};
};

class CurrentSceneHook final : public IPlanningSceneRevisionHook
{
public:
  SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken &) override
  {
    calls_.fetch_add(1U);
    return {SceneRevisionState::kCurrent, expected, "current"};
  }

  [[nodiscard]] std::size_t calls() const noexcept
  {
    return calls_.load();
  }

private:
  std::atomic<std::size_t> calls_{0U};
};

class StaleSceneHook final : public IPlanningSceneRevisionHook
{
public:
  SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken &) override
  {
    auto observed = expected;
    ++observed.revision;
    return {
      SceneRevisionState::kRefreshRequired, std::move(observed),
      "authoritative scene advanced"};
  }
};

class RecordingBackoff final : public IPlanningBackoff
{
public:
  PlanningStopCause wait(
    const std::chrono::steady_clock::duration duration,
    const PlanningStopToken & stop_token) override
  {
    durations.push_back(duration);
    return stop_token.cause();
  }

  std::vector<std::chrono::steady_clock::duration> durations;
};

class BlockingBackoff final : public IPlanningBackoff
{
public:
  PlanningStopCause wait(
    const std::chrono::steady_clock::duration,
    const PlanningStopToken & stop_token) override
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      entered_ = true;
      cv_.notify_all();
    }
    return stop_token.wait_for(5s);
  }

  bool wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 1s, [this]() {return entered_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool entered_{false};
};

TEST(ArmPlanningPortTest, RetryableBudgetBacksOffThenSucceeds)
{
  ScriptedBackend backend({
        ArmPlanningResult::failure(
        PlanningOutcome::kBudgetExhausted, "attempt budget elapsed", -6),
        success(),
      });
  CurrentSceneHook scene;
  RecordingBackoff backoff;
  const PlanningRetryPolicy policy{3U, 4ms, 10ms};
  ArmPlanningPort port(backend, scene, policy, backoff);

  const auto result = port.plan(request_for());

  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(result.attempt_count(), 2U);
  EXPECT_EQ(result.planned_scene_revision(), revision());
  EXPECT_EQ(backend.calls(), 2U);
  EXPECT_EQ(scene.calls(), 2U);
  ASSERT_EQ(backoff.durations.size(), 1U);
  EXPECT_EQ(backoff.durations.front(), 4ms);
}

TEST(ArmPlanningPortTest, DefaultRetryBackoffServicesExecutionContext)
{
  ScriptedBackend backend({
        ArmPlanningResult::failure(
          PlanningOutcome::kBudgetExhausted, "attempt budget elapsed", -6),
        success(),
      });
  CurrentSceneHook scene;
  ArmPlanningPort port(backend, scene, {2U, 12ms, 12ms});
  CountingExecutionContext context;

  const auto result = port.plan(request_for(), context);

  ASSERT_TRUE(result.succeeded()) << result.detail();
  EXPECT_EQ(result.attempt_count(), 2U);
  EXPECT_GE(context.checkpoints.load(), 6U);
  EXPECT_GE(context.heartbeats.load(), 2U);
}

TEST(ArmPlanningPortTest, NonRetryableFailureDoesNotRetry)
{
  ScriptedBackend backend({
        ArmPlanningResult::failure(
        PlanningOutcome::kInvalidGoalState, "goal is in collision", -12),
        success(),
      });
  CurrentSceneHook scene;
  RecordingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);

  const auto result = port.plan(request_for());

  EXPECT_EQ(result.outcome(), PlanningOutcome::kInvalidGoalState);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kNonRetryable);
  EXPECT_EQ(result.attempt_count(), 1U);
  EXPECT_EQ(backend.calls(), 1U);
  EXPECT_EQ(scene.calls(), 1U);
  EXPECT_TRUE(backoff.durations.empty());
}

TEST(ArmPlanningPortTest, CancelDuringBackoffPreventsNextAttempt)
{
  ScriptedBackend backend({
        ArmPlanningResult::failure(
        PlanningOutcome::kBudgetExhausted, "attempt budget elapsed", -6),
        success(),
      });
  CurrentSceneHook scene;
  BlockingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);

  auto result_future = std::async(
    std::launch::async, [&port]() {return port.plan(request_for());});
  ASSERT_TRUE(backoff.wait_until_entered());
  port.request_stop(PlanningStopCause::kCanceled);
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();

  EXPECT_EQ(result.outcome(), PlanningOutcome::kCanceled);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kStopped);
  EXPECT_EQ(result.attempt_count(), 1U);
  EXPECT_EQ(backend.calls(), 1U);
  EXPECT_EQ(scene.calls(), 1U);
  EXPECT_EQ(backend.stop_calls(), 1U);
}

TEST(ArmPlanningPortTest, ShutdownDuringBackoffMapsTypedOutcomeAndPreventsNextAttempt)
{
  ScriptedBackend backend({
        ArmPlanningResult::failure(
          PlanningOutcome::kBudgetExhausted, "attempt budget elapsed", -6),
        success(),
      });
  CurrentSceneHook scene;
  BlockingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);

  auto result_future = std::async(
    std::launch::async, [&port]() {return port.plan(request_for());});
  ASSERT_TRUE(backoff.wait_until_entered());
  port.request_stop(PlanningStopCause::kShutdown);
  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  const auto result = result_future.get();

  EXPECT_EQ(result.outcome(), PlanningOutcome::kShutdown);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kStopped);
  EXPECT_EQ(result.attempt_count(), 1U);
  EXPECT_EQ(backend.calls(), 1U);
  EXPECT_EQ(scene.calls(), 1U);
  EXPECT_EQ(backend.stop_calls(), 1U);
}

TEST(ArmPlanningPortTest, StaleSceneForcesRefreshBeforeAnyBackendSideEffect)
{
  ScriptedBackend backend({success()});
  StaleSceneHook scene;
  RecordingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);

  const auto result = port.plan(request_for(revision(11U)));

  EXPECT_EQ(result.outcome(), PlanningOutcome::kSceneRefreshRequired);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kNonRetryable);
  EXPECT_EQ(result.attempt_count(), 0U);
  EXPECT_EQ(result.planned_scene_revision(), revision(12U));
  EXPECT_EQ(backend.calls(), 0U);
  EXPECT_TRUE(backoff.durations.empty());
}

TEST(ArmPlanningPortTest, SceneDiffIsRejectedBeforeRevisionOrBackendSideEffects)
{
  ScriptedBackend backend({success()});
  CurrentSceneHook scene;
  RecordingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);
  auto request = request_for();
  request.scene_snapshot.is_diff = true;

  const auto result = port.plan(request);

  EXPECT_EQ(result.outcome(), PlanningOutcome::kInvalidRequest);
  EXPECT_EQ(result.attempt_count(), 0U);
  EXPECT_EQ(scene.calls(), 0U);
  EXPECT_EQ(backend.calls(), 0U);
  EXPECT_TRUE(backoff.durations.empty());
}

TEST(ArmPlanningPortTest, SuccessfulTrajectoryMustStartAtSnapshotRobotState)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_1"};
  trajectory.joint_trajectory.points.emplace_back();
  trajectory.joint_trajectory.points.front().positions = {0.3};
  ScriptedBackend backend({ArmPlanningResult::success(std::move(trajectory), "jump")});
  CurrentSceneHook scene;
  RecordingBackoff backoff;
  ArmPlanningPort port(backend, scene, {3U, 1ms, 4ms}, backoff);

  const auto result = port.plan(request_for());

  EXPECT_EQ(result.outcome(), PlanningOutcome::kInvalidStartState);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kNonRetryable);
  EXPECT_EQ(result.attempt_count(), 1U);
  EXPECT_EQ(backend.calls(), 1U);
  EXPECT_TRUE(backoff.durations.empty());
}

TEST(TrajectoryStateValidationTest, RejectsNonUnitMultiDofQuaternion)
{
  moveit_msgs::msg::RobotState state;
  state.multi_dof_joint_state.joint_names = {"floating_joint"};
  state.multi_dof_joint_state.transforms.emplace_back();
  state.multi_dof_joint_state.transforms.front().rotation.w = 1.0;
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.multi_dof_joint_trajectory.joint_names = {"floating_joint"};
  trajectory.multi_dof_joint_trajectory.points.emplace_back();
  trajectory.multi_dof_joint_trajectory.points.front().transforms.emplace_back();
  trajectory.multi_dof_joint_trajectory.points.front().transforms.front().rotation.w = 2.0;

  const auto result = detail::check_trajectory_start_state(state, trajectory, 0.01);

  EXPECT_EQ(result.status, detail::TrajectoryStartStateStatus::kInvalid);
}

TEST(TrajectoryStateValidationTest, UsesAngularDistanceForMultiDofTolerance)
{
  moveit_msgs::msg::RobotState state;
  state.multi_dof_joint_state.joint_names = {"floating_joint"};
  state.multi_dof_joint_state.transforms.emplace_back();
  state.multi_dof_joint_state.transforms.front().rotation.w = 1.0;
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.multi_dof_joint_trajectory.joint_names = {"floating_joint"};
  trajectory.multi_dof_joint_trajectory.points.emplace_back();
  trajectory.multi_dof_joint_trajectory.points.front().transforms.emplace_back();
  const double half_angle = 0.05;
  auto & planned = trajectory.multi_dof_joint_trajectory.points.front().transforms.front();
  planned.rotation.z = std::sin(half_angle);
  planned.rotation.w = std::cos(half_angle);

  const auto result = detail::check_trajectory_start_state(state, trajectory, 0.01);

  EXPECT_EQ(result.status, detail::TrajectoryStartStateStatus::kMismatch);
}

}  // namespace
}  // namespace manipulation::moveit
