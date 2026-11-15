// Responsibility: Verify the MoveIt adapter preserves a full scene and only requests planning.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/allowed_collision_entry.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "action_test_support.hpp"
#include "action_call.hpp"
#include "manipulation_moveit/moveit_arm_planning_adapter.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using MoveGroup = moveit_msgs::action::MoveGroup;
using MoveGroupHandle = rclcpp_action::ServerGoalHandle<MoveGroup>;
using test::ExecutorThread;
using RclcppFixture = test::RclcppFixture;

class CurrentSceneHook final : public IPlanningSceneRevisionHook
{
public:
  SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken &) override
  {
    return {SceneRevisionState::kCurrent, expected, "current"};
  }
};

class PlanningTestContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {return !paused_ || stopped_.load();});
    return stopped_.load() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load();
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return checkpoint();
  }

  manipulation::workflow::TransferExecutionControlResult heartbeat() override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++heartbeats_;
    cv_.notify_all();
    return stopped_.load() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  manipulation::workflow::TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    gate_entered_ = true;
    cv_.notify_all();
    if (block_dispatch_) {
      cv_.wait(lock, [this]() {return !block_dispatch_ || stopped_.load();});
    }
    if (stopped_.load()) {
      return manipulation::workflow::TaskOperationGateResult::kStopped;
    }
    if (pause_next_dispatch_) {
      pause_next_dispatch_ = false;
      paused_ = true;
      cv_.notify_all();
      return manipulation::workflow::TaskOperationGateResult::kPaused;
    }
    if (paused_) {
      return manipulation::workflow::TaskOperationGateResult::kPaused;
    }
    // Deliberately invoke under a non-recursive mutex. Production dispatch
    // closures must not re-enter execution-context lifecycle methods.
    operation();
    return manipulation::workflow::TaskOperationGateResult::kExecuted;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void block_dispatch()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    block_dispatch_ = true;
  }

  void pause_next_dispatch()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_next_dispatch_ = true;
  }

  void cancel()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stopped_.store(true);
    block_dispatch_ = false;
    cv_.notify_all();
  }

  void resume()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    paused_ = false;
    block_dispatch_ = false;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_for_gate(
    const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return gate_entered_;});
  }

  [[nodiscard]] bool wait_for_heartbeats(
    const std::size_t count,
    const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this, count]() {return heartbeats_ >= count;});
  }

private:
  std::atomic_bool stopped_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t heartbeats_{0U};
  bool gate_entered_{false};
  bool block_dispatch_{false};
  bool pause_next_dispatch_{false};
  bool paused_{false};
};

MoveItArmPlanningAdapterConfig config(const std::string & action_name)
{
  MoveItArmPlanningAdapterConfig result;
  result.move_group_action = action_name;
  result.planning_group = "manipulator";
  result.end_effector_link = "tool_frame";
  result.action_poll_period = 1ms;
  return result;
}

moveit_msgs::msg::PlanningScene complete_scene()
{
  moveit_msgs::msg::PlanningScene scene;
  scene.name = "authoritative-scene";
  scene.robot_model_name = "test-robot";
  scene.is_diff = false;
  scene.robot_state.is_diff = false;

  moveit_msgs::msg::CollisionObject world_object;
  world_object.header.frame_id = "world";
  world_object.id = "table";
  world_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.world.collision_objects.push_back(world_object);

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool_frame";
  attached_object.object.header.frame_id = "tool_frame";
  attached_object.object.id = "held-object";
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.touch_links = {"finger_left", "finger_right"};
  scene.robot_state.attached_collision_objects.push_back(attached_object);

  scene.allowed_collision_matrix.entry_names = {"held-object", "finger_left"};
  moveit_msgs::msg::AllowedCollisionEntry held_row;
  held_row.enabled = {false, true};
  moveit_msgs::msg::AllowedCollisionEntry finger_row;
  finger_row.enabled = {true, false};
  scene.allowed_collision_matrix.entry_values = {held_row, finger_row};

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "camera_mount";
  transform.transform.translation.x = 1.25;
  transform.transform.rotation.w = 1.0;
  scene.fixed_frame_transforms.push_back(transform);
  return scene;
}

ArmPlanningRequest request_with(const moveit_msgs::msg::PlanningScene & scene)
{
  ArmPlanningRequest request;
  request.goal_kind = ArmPlanningGoalKind::kPose;
  request.target_pose.header.frame_id = "world";
  request.target_pose.header.stamp.sec = 1;
  request.target_pose.pose.position.x = 0.4;
  request.target_pose.pose.orientation.w = 1.0;
  request.label = "pregrasp";
  request.scene_revision = {"scene-session", 42U};
  request.scene_snapshot = scene;
  return request;
}

ArmPlanningRequest request_with_start_state()
{
  auto scene = complete_scene();
  scene.robot_state.joint_state.name = {"joint_1"};
  scene.robot_state.joint_state.position = {0.0};
  return request_with(scene);
}

std::shared_ptr<MoveGroup::Result> successful_plan()
{
  auto result = std::make_shared<MoveGroup::Result>();
  result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0};
  result->planned_trajectory.joint_trajectory.joint_names = {"joint_1"};
  result->planned_trajectory.joint_trajectory.points.push_back(std::move(point));
  return result;
}

TEST(
  MoveItArmPlanningAdapterPolicy,
  DiscoveryGoalResponseAndResultShareUnboundedTransportDeadline)
{
  const auto deadline = detail::unbounded_action_deadline();

  EXPECT_EQ(deadline, std::chrono::steady_clock::time_point::max());
  EXPECT_GT(deadline - std::chrono::steady_clock::now(), 60s);
}

TEST_F(RclcppFixture, CancelAtMoveGroupDispatchGateSendsNoGoal)
{
  const std::string action_name = "/test_moveit_arm_planning/cancel_at_dispatch";
  auto server_node = rclcpp::Node::make_shared("planning_cancel_dispatch_server");
  auto client_node = rclcpp::Node::make_shared("planning_cancel_dispatch_client");
  std::atomic_size_t received_goals{0U};
  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [&received_goals](
      const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveGroup::Goal>) {
      received_goals.fetch_add(1U);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<MoveGroupHandle> handle) {
      handle->succeed(successful_plan());
    });
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));
  CurrentSceneHook scene_hook;
  ArmPlanningPort port(adapter, scene_hook, {1U, 1ms, 1ms});
  PlanningTestContext context;
  context.block_dispatch();

  auto future = std::async(std::launch::async, [&]() {
        return port.plan(request_with_start_state(), context);
    });
  ASSERT_TRUE(context.wait_for_gate(1s));
  context.cancel();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);

  EXPECT_EQ(future.get().outcome(), PlanningOutcome::kCanceled);
  EXPECT_EQ(received_goals.load(), 0U);
  (void)server;
}

TEST_F(RclcppFixture, PauseAtMoveGroupDispatchGateDefersGoalUntilResume)
{
  const std::string action_name = "/test_moveit_arm_planning/pause_at_dispatch";
  auto server_node = rclcpp::Node::make_shared("planning_pause_dispatch_server");
  auto client_node = rclcpp::Node::make_shared("planning_pause_dispatch_client");
  std::atomic_size_t received_goals{0U};
  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [&received_goals](
      const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveGroup::Goal>) {
      received_goals.fetch_add(1U);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<MoveGroupHandle> handle) {
      handle->succeed(successful_plan());
    });
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));
  CurrentSceneHook scene_hook;
  ArmPlanningPort port(adapter, scene_hook, {1U, 1ms, 1ms});
  PlanningTestContext context;
  context.pause_next_dispatch();

  auto future = std::async(std::launch::async, [&]() {
        return port.plan(request_with_start_state(), context);
    });
  ASSERT_TRUE(context.wait_for_gate(1s));
  EXPECT_EQ(received_goals.load(), 0U);
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);

  context.resume();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get().succeeded());
  EXPECT_EQ(received_goals.load(), 1U);
  (void)server;
}

TEST_F(RclcppFixture, MoveGroupResultWaitServicesExecutionHeartbeat)
{
  const std::string action_name = "/test_moveit_arm_planning/result_heartbeat";
  auto server_node = rclcpp::Node::make_shared("planning_heartbeat_server");
  auto client_node = rclcpp::Node::make_shared("planning_heartbeat_client");
  std::promise<std::shared_ptr<MoveGroupHandle>> accepted;
  auto accepted_future = accepted.get_future();
  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveGroup::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&accepted](const std::shared_ptr<MoveGroupHandle> handle) {
      accepted.set_value(handle);
    });
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));
  CurrentSceneHook scene_hook;
  ArmPlanningPort port(adapter, scene_hook, {1U, 1ms, 1ms});
  PlanningTestContext context;

  auto future = std::async(std::launch::async, [&]() {
        return port.plan(request_with_start_state(), context);
    });
  ASSERT_EQ(accepted_future.wait_for(1s), std::future_status::ready);
  const auto handle = accepted_future.get();
  ASSERT_TRUE(context.wait_for_heartbeats(5U, 1s));
  handle->succeed(successful_plan());

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get().succeeded());
  (void)server;
}

TEST_F(RclcppFixture, AdapterPreservesCompleteSceneAndMapsInternalBudgetTimeout)
{
  const std::string action_name = "/test_moveit_arm_planning/full_scene";
  auto server_node = rclcpp::Node::make_shared("planning_snapshot_server");
  auto client_node = rclcpp::Node::make_shared("planning_snapshot_client");
  std::promise<MoveGroup::Goal> captured_goal;
  auto captured_goal_future = captured_goal.get_future();
  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [&captured_goal](
      const rclcpp_action::GoalUUID &,
      const std::shared_ptr<const MoveGroup::Goal> goal)
    {
      captured_goal.set_value(*goal);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<MoveGroupHandle> handle) {
      auto result = std::make_shared<MoveGroup::Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      result->error_code.message = "internal planner budget elapsed";
      handle->abort(std::move(result));
    });
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));
  const auto readiness_deadline = std::chrono::steady_clock::now() + 1s;
  while (!adapter.cached_readiness().ready &&
    std::chrono::steady_clock::now() < readiness_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(adapter.cached_readiness().ready);
  const auto expected_scene = complete_scene();
  auto request = request_with(expected_scene);
  request.target_pose.header.frame_id = "panda_link0";

  const auto result = adapter.plan_once(
    request, PlanningStopToken{});

  EXPECT_EQ(result.outcome(), PlanningOutcome::kBudgetExhausted);
  EXPECT_EQ(result.failure_class(), PlanningFailureClass::kRetryable);
  EXPECT_EQ(result.backend_error_code(), moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT);
  ASSERT_EQ(captured_goal_future.wait_for(1s), std::future_status::ready);
  const auto goal = captured_goal_future.get();
  EXPECT_TRUE(goal.planning_options.plan_only);
  EXPECT_FALSE(goal.planning_options.planning_scene_diff.is_diff);
  EXPECT_FALSE(goal.planning_options.planning_scene_diff.robot_state.is_diff);
  EXPECT_DOUBLE_EQ(goal.request.allowed_planning_time, 5.0);
  EXPECT_EQ(goal.request.num_planning_attempts, 1);
  ASSERT_EQ(goal.request.goal_constraints.size(), 1U);
  const auto & constraints = goal.request.goal_constraints.front();
  ASSERT_EQ(constraints.position_constraints.size(), 1U);
  EXPECT_EQ(constraints.position_constraints.front().header.frame_id, "panda_link0");
  ASSERT_EQ(constraints.orientation_constraints.size(), 1U);
  EXPECT_EQ(constraints.orientation_constraints.front().header.frame_id, "panda_link0");
  ASSERT_EQ(
    goal.planning_options.planning_scene_diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(
    goal.planning_options.planning_scene_diff.world.collision_objects.front().header.frame_id,
    "world");
  EXPECT_EQ(
    goal.planning_options.planning_scene_diff.world.collision_objects,
    expected_scene.world.collision_objects);
  EXPECT_EQ(
    goal.planning_options.planning_scene_diff.robot_state.attached_collision_objects,
    expected_scene.robot_state.attached_collision_objects);
  EXPECT_EQ(
    goal.planning_options.planning_scene_diff.allowed_collision_matrix,
    expected_scene.allowed_collision_matrix);
  EXPECT_EQ(
    goal.planning_options.planning_scene_diff.fixed_frame_transforms,
    expected_scene.fixed_frame_transforms);
  (void)server;
}

TEST_F(RclcppFixture, DiscoveryDelayDoesNotBecomeAPlanningFailure)
{
  const std::string action_name = "/test_moveit_arm_planning/delayed_discovery";
  auto server_node = rclcpp::Node::make_shared("planning_delayed_server");
  auto client_node = rclcpp::Node::make_shared("planning_delayed_client");
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));

  auto result_future = std::async(
    std::launch::async,
    [&adapter]() {
      return adapter.plan_once(request_with(complete_scene()), PlanningStopToken{});
    });
  EXPECT_EQ(result_future.wait_for(30ms), std::future_status::timeout);

  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveGroup::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<MoveGroupHandle> handle) {
      auto result = std::make_shared<MoveGroup::Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {0.0};
      result->planned_trajectory.joint_trajectory.joint_names = {"joint_1"};
      result->planned_trajectory.joint_trajectory.points.push_back(std::move(point));
      handle->succeed(std::move(result));
    });

  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(result_future.get().succeeded());
  (void)server;
}

TEST_F(RclcppFixture, CancelReturnsWithoutWaitingForLateMoveGroupTerminalResult)
{
  const std::string action_name = "/test_moveit_arm_planning/cancel";
  auto server_node = rclcpp::Node::make_shared("planning_cancel_server");
  auto client_node = rclcpp::Node::make_shared("planning_cancel_client");
  std::promise<std::shared_ptr<MoveGroupHandle>> accepted_goal;
  auto accepted_goal_future = accepted_goal.get_future();
  auto server = rclcpp_action::create_server<MoveGroup>(
    server_node, action_name,
    [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const MoveGroup::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<MoveGroupHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&accepted_goal](const std::shared_ptr<MoveGroupHandle> handle) {
      accepted_goal.set_value(handle);
    });
  ExecutorThread executor{server_node, client_node};
  MoveItArmPlanningAdapter adapter(*client_node, config(action_name));
  CurrentSceneHook scene_hook;
  ArmPlanningPort port(adapter, scene_hook, {2U, 1ms, 2ms});
  const auto readiness_deadline = std::chrono::steady_clock::now() + 1s;
  while (!adapter.cached_readiness().ready &&
    std::chrono::steady_clock::now() < readiness_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(adapter.cached_readiness().ready);
  auto result_future = std::async(
    std::launch::async,
    [&port]() {return port.plan(request_with(complete_scene()));});
  ASSERT_EQ(accepted_goal_future.wait_for(1s), std::future_status::ready);
  const auto handle = accepted_goal_future.get();

  port.request_stop(PlanningStopCause::kCanceled);

  ASSERT_EQ(result_future.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(result_future.get().outcome(), PlanningOutcome::kCanceled);
  EXPECT_FALSE(adapter.cached_readiness().ready);

  const auto cancel_deadline = std::chrono::steady_clock::now() + 1s;
  while (!handle->is_canceling() && std::chrono::steady_clock::now() < cancel_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(handle->is_canceling());
  auto canceled = std::make_shared<MoveGroup::Result>();
  canceled->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
  handle->canceled(std::move(canceled));
  const auto terminal_deadline = std::chrono::steady_clock::now() + 1s;
  while (!adapter.cached_readiness().ready &&
    std::chrono::steady_clock::now() < terminal_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(adapter.cached_readiness().ready);
  (void)server;
}

}  // namespace
}  // namespace manipulation::moveit
