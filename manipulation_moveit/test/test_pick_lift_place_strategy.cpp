// Responsibility: Verifies the backend-neutral production pick/lift/place protocol on CPU fakes.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/scene_lease_release_policy.hpp"
#include "manipulation/workflow/transfer_workflow_types.hpp"

#include "manipulation_moveit/arm_planning_port.hpp"
#include "manipulation_moveit/pick_lift_place_strategy.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using ControlResult = manipulation::workflow::TransferExecutionControlResult;
using RecoveryPolicy = manipulation::workflow::TransferRecoveryPolicy;
using ResolvedTask = manipulation::workflow::ResolvedTransferTask;
using SceneLease = manipulation::workflow::PlanningSceneLease;
using WaitingReason = manipulation::workflow::TransferWaitingReason;

class EventLog final
{
public:
  void add(std::string event)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(std::move(event));
  }

  [[nodiscard]] std::vector<std::string> snapshot() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::string> events_;
};

builtin_interfaces::msg::Time test_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123U;
  return stamp;
}

geometry_msgs::msg::Pose rigid_pose(
  const double x = 0.0,
  const double y = 0.0,
  const double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::PoseStamped stamped_pose(const double x)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = test_stamp();
  pose.pose = rigid_pose(x);
  return pose;
}

manipulation::workflow::PlanningSceneSnapshot scene_snapshot(
  const std::uint64_t revision,
  const double joint_position = 0.0)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.name = "strategy-test-scene";
  scene.is_diff = false;
  scene.robot_state.is_diff = false;
  scene.robot_state.joint_state.name = {"joint_1"};
  scene.robot_state.joint_state.position = {joint_position};
  return manipulation::workflow::PlanningSceneSnapshot::create(
    "scene-session", revision, std::move(scene));
}

manipulation::workflow::PlanningSceneAccessProof scene_proof(
  const std::uint64_t revision)
{
  return manipulation::workflow::PlanningSceneAccessProof::create(
    "lease-1", "scene-session", revision);
}

ResolvedTask resolved_task(
  const bool return_home = false,
  const RecoveryPolicy recovery = RecoveryPolicy::kNone)
{
  auto observation = manipulation::workflow::ObjectObservation::create(
    "cube_1", stamped_pose(0.0), "observation-1", 0.9);
  auto pregrasp = stamped_pose(-0.1);
  auto grasp_pose = stamped_pose(0.0);
  geometry_msgs::msg::Vector3Stamped approach;
  approach.header = grasp_pose.header;
  approach.vector.x = 1.0;
  auto grasp = manipulation::workflow::GraspCandidate::create(
    observation.correlation(), std::move(pregrasp), std::move(grasp_pose),
    std::move(approach), 0.04, 20.0, 0.8);
  auto instance = manipulation::workflow::ObjectInstance::create(
    "cube_1", "cube_profile");
  auto profile = manipulation::workflow::ObjectProfile::create(
    "cube_profile",
    manipulation::workflow::ObjectGeometry::primitive(
      manipulation::workflow::PrimitiveGeometry::create(
        manipulation::workflow::PrimitiveGeometryType::kBox,
        {0.1, 0.2, 0.3}, rigid_pose())));
  return ResolvedTask::create(
    std::move(instance), std::move(profile), std::move(observation), std::move(grasp),
    scene_snapshot(7U), scene_proof(7U), stamped_pose(0.5), return_home, recovery);
}

PickLiftPlaceStrategyConfig valid_config()
{
  PickLiftPlaceStrategyConfig config;
  config.robot.attachment_link = "tool0";
  config.robot.touch_links = {"tool0", "finger_left", "finger_right"};
  config.robot.open_width_m = 0.08;
  config.robot.allow_grasp_stall = true;
  config.robot.home_joint_names = {"joint_1", "joint_2"};
  config.robot.home_joint_positions = {0.0, 0.5};
  config.task.lift_clearance_m = 0.1;
  config.task.preplace_clearance_m = 0.12;
  config.task.retreat_clearance_m = 0.15;
  config.task.release_settle = 0ms;
  config.arm_motion_timeout = 3s;
  config.gripper_command_timeout = 2s;
  return config;
}

class FakeContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  ControlResult checkpoint() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++checkpoint_count_;
    if (stopped_.load()) {
      return ControlResult::kStopped;
    }
    if (pause_at_checkpoint_ == checkpoint_count_) {
      paused_ = true;
      cv_.notify_all();
      cv_.wait(lock, [this]() {return resumed_ || stopped_.load();});
    }
    return stopped_.load() ? ControlResult::kStopped : ControlResult::kContinue;
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load();
  }

  ControlResult wait_until(TimePoint) override
  {
    if (stop_on_wait_) {
      stopped_.store(true);
    }
    return stopped_.load() ? ControlResult::kStopped : ControlResult::kContinue;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot feedback) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    feedback_.push_back(std::move(feedback));
  }

  void report_waiting(
    const WaitingReason reason,
    manipulation::task::TaskStateSnapshot feedback) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    waiting_reasons_.push_back(reason);
    feedback_.push_back(std::move(feedback));
  }

  void cancel()
  {
    stopped_.store(true);
    cv_.notify_all();
  }

  void stop_on_wait()
  {
    stop_on_wait_ = true;
  }

  void pause_on_checkpoint(const std::size_t checkpoint)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_at_checkpoint_ = checkpoint;
  }

  [[nodiscard]] bool wait_until_paused(const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return paused_;});
  }

  void resume()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    resumed_ = true;
    cv_.notify_all();
  }

  [[nodiscard]] std::vector<WaitingReason> waiting_reasons() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return waiting_reasons_;
  }

  [[nodiscard]] std::vector<manipulation::task::TaskStateSnapshot> feedback() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return feedback_;
  }

private:
  std::atomic_bool stopped_{false};
  bool stop_on_wait_{false};
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t checkpoint_count_{0U};
  std::size_t pause_at_checkpoint_{0U};
  bool paused_{false};
  bool resumed_{false};
  std::vector<WaitingReason> waiting_reasons_;
  std::vector<manipulation::task::TaskStateSnapshot> feedback_;
};

class FakePlanningBackend final : public IArmPlanningBackend
{
public:
  explicit FakePlanningBackend(EventLog & log)
  : log_(log)
  {
  }

  [[nodiscard]] ArmPlanningReadiness cached_readiness() const override
  {
    return {true, "ready"};
  }

  ArmPlanningResult plan_once(
    const ArmPlanningRequest & request,
    const PlanningStopToken & stop_token) override
  {
    log_.add("plan:" + request.label);
    requests.push_back(request);
    if (stop_token.stop_requested()) {
      return ArmPlanningResult::failure(PlanningOutcome::kCanceled, "stopped");
    }
    if (budget_failures_remaining > 0U) {
      --budget_failures_remaining;
      return ArmPlanningResult::failure(
        PlanningOutcome::kBudgetExhausted, "planner budget exhausted");
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1"};
    const auto start = request.scene_snapshot.robot_state.joint_state.position.front();
    trajectory_msgs::msg::JointTrajectoryPoint start_point;
    start_point.positions = {start};
    trajectory.joint_trajectory.points.push_back(std::move(start_point));
    trajectory_msgs::msg::JointTrajectoryPoint terminal_point;
    terminal_point.positions = {start + 0.1};
    trajectory.joint_trajectory.points.push_back(std::move(terminal_point));
    return ArmPlanningResult::success(std::move(trajectory), "planned");
  }

  void request_stop() noexcept override
  {
    ++stop_requests;
  }

  std::size_t budget_failures_remaining{0U};
  std::size_t stop_requests{0U};
  std::vector<ArmPlanningRequest> requests;

private:
  EventLog & log_;
};

class CurrentRevisionHook final : public IPlanningSceneRevisionHook
{
public:
  SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken & stop_token) override
  {
    if (stop_token.stop_requested()) {
      return {SceneRevisionState::kUnavailable, {}, "stopped"};
    }
    return {SceneRevisionState::kCurrent, expected, "current"};
  }
};

class ImmediateBackoff final : public IPlanningBackoff
{
public:
  PlanningStopCause wait(
    std::chrono::steady_clock::duration,
    const PlanningStopToken & stop_token) override
  {
    return stop_token.cause();
  }
};

class FakeTrajectoryPort final : public ITrajectoryExecutionPort
{
public:
  explicit FakeTrajectoryPort(EventLog & log)
  : log_(log)
  {
  }

  void begin_goal() override {}

  PhysicalCommandResult execute(
    const TrajectoryExecutionRequest & request,
    const std::chrono::steady_clock::duration accepted_motion_timeout,
    manipulation::workflow::TransferExecutionContext &) override
  {
    log_.add("trajectory:" + request.label);
    observed_timeouts.push_back(accepted_motion_timeout);
    if (on_execute) {
      on_execute(request);
    }
    if (failure && request.label == failure_label) {
      auto result = *failure;
      failure.reset();
      return result;
    }
    if (on_success) {
      on_success();
    }
    return PhysicalCommandResult::success("executed");
  }

  void request_stop() noexcept override
  {
    ++stop_requests;
  }

  std::string failure_label;
  std::optional<PhysicalCommandResult> failure;
  std::function<void(const TrajectoryExecutionRequest &)> on_execute;
  std::function<void()> on_success;
  std::vector<std::chrono::steady_clock::duration> observed_timeouts;
  std::size_t stop_requests{0U};

private:
  EventLog & log_;
};

class FakeGripperPort final : public IStrategyGripperCommandPort
{
public:
  explicit FakeGripperPort(EventLog & log)
  : log_(log)
  {
  }

  void begin_goal() override {}

  StrategyGripperCommandResult command(
    const StrategyGripperCommandRequest & request,
    const std::chrono::steady_clock::duration accepted_command_timeout,
    manipulation::workflow::TransferExecutionContext &) override
  {
    log_.add("gripper:" + request.label);
    requests.push_back(request);
    observed_timeouts.push_back(accepted_command_timeout);
    if (on_command) {
      on_command(request);
    }
    if (failure && request.label == failure_label) {
      auto result = *failure;
      failure.reset();
      return result;
    }
    return StrategyGripperCommandResult::success(
      request.kind == GripperCommandKind::kOpen ?
      GripperPostcondition::kOpenConfirmed : GripperPostcondition::kClosedConfirmed,
      "confirmed");
  }

  void request_stop() noexcept override
  {
    ++stop_requests;
  }

  std::string failure_label;
  std::optional<StrategyGripperCommandResult> failure;
  std::function<void(const StrategyGripperCommandRequest &)> on_command;
  std::vector<StrategyGripperCommandRequest> requests;
  std::vector<std::chrono::steady_clock::duration> observed_timeouts;
  std::size_t stop_requests{0U};

private:
  EventLog & log_;
};

class FakeScenePort final : public IPickLiftPlaceScenePort
{
public:
  explicit FakeScenePort(EventLog & log)
  : log_(log)
  {
  }

  void reset()
  {
    observed_robot_position_ = 0.0;
    current_ = SceneLease::create(
      scene_snapshot(7U, observed_robot_position_), scene_proof(7U));
  }

  void begin_goal() override {}

  CoordinatedSceneResult revalidate(
    const StrategySceneRevalidationRequest & request,
    manipulation::workflow::TransferExecutionContext &) override
  {
    log_.add("scene:revalidate");
    request_evidence_consistent = request_evidence_consistent &&
      request.scene_snapshot.coordinator_session_id() == "scene-session" &&
      request.access_proof.coordinator_session_id() == "scene-session" &&
      request.scene_snapshot.revision() == request.access_proof.revision() &&
      !request.scene_snapshot.scene().is_diff &&
      !request.scene_snapshot.scene().robot_state.is_diff;
    if (on_revalidate) {
      on_revalidate();
    }
    if (revision_mismatch) {
      const auto next_revision = current_->snapshot().revision() + 1U;
      return CoordinatedSceneResult::success(
        SceneLease::create(
          scene_snapshot(next_revision, observed_robot_position_), scene_proof(next_revision)),
        "unexpected newer revision");
    }
    if (refresh_robot_state) {
      const auto revision = current_->snapshot().revision();
      current_ = SceneLease::create(
        scene_snapshot(revision, observed_robot_position_), scene_proof(revision));
    }
    return CoordinatedSceneResult::success(*current_, "current");
  }

  CoordinatedSceneResult attach(
    const StrategySceneAttachRequest &,
    manipulation::workflow::TransferExecutionContext &) override
  {
    log_.add("scene:attach");
    if (on_attach) {
      on_attach();
    }
    if (attach_failure) {
      return CoordinatedSceneResult::failure(
        *attach_failure, "injected attach failure");
    }
    return advance("attached");
  }

  CoordinatedSceneResult detach(
    const StrategySceneDetachRequest &,
    manipulation::workflow::TransferExecutionContext &) override
  {
    log_.add("scene:detach");
    if (on_detach) {
      on_detach();
    }
    if (detach_failure) {
      return CoordinatedSceneResult::failure(
        *detach_failure, "injected detach failure");
    }
    return advance("detached");
  }

  void request_stop() noexcept override
  {
    ++stop_requests;
  }

  bool revision_mismatch{false};
  bool refresh_robot_state{false};
  bool request_evidence_consistent{true};
  std::function<void()> on_revalidate;
  std::function<void()> on_attach;
  std::function<void()> on_detach;
  std::optional<CoordinatedSceneOutcome> attach_failure;
  std::optional<CoordinatedSceneOutcome> detach_failure;
  std::size_t stop_requests{0U};

  void advance_observed_robot_state()
  {
    observed_robot_position_ += 0.1;
  }

private:
  CoordinatedSceneResult advance(std::string detail)
  {
    const auto next_revision = current_->snapshot().revision() + 1U;
    current_ = SceneLease::create(
      scene_snapshot(next_revision, observed_robot_position_), scene_proof(next_revision));
    return CoordinatedSceneResult::success(*current_, std::move(detail));
  }

  EventLog & log_;
  std::optional<SceneLease> current_;
  double observed_robot_position_{0.0};
};

class FakeHealthPort final : public IPickLiftPlaceHealthPort
{
public:
  explicit FakeHealthPort(EventLog & log)
  : log_(log)
  {
  }

  void begin_goal() override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = false;
    entered_ = false;
  }

  PickLiftPlaceHealthResult wait_until_ready(
    manipulation::workflow::TransferExecutionContext & context) override
  {
    log_.add("health:wait");
    std::unique_lock<std::mutex> lock(mutex_);
    ++wait_count_;
    entered_ = true;
    cv_.notify_all();
    if (block_) {
      cv_.wait(lock, [this, &context]() {
          return stopped_ || !block_ || context.stop_requested();
      });
    }
    if (stopped_ || context.stop_requested()) {
      return PickLiftPlaceHealthResult::failure(
        PickLiftPlaceHealthOutcome::kCanceled, "health wait canceled");
    }
    return PickLiftPlaceHealthResult::ready("ready");
  }

  void request_stop() noexcept override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    ++stop_requests_;
    cv_.notify_all();
  }

  void set_blocked(const bool blocked)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    block_ = blocked;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_until_entered(const std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return entered_;});
  }

  [[nodiscard]] std::size_t wait_count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return wait_count_;
  }

private:
  EventLog & log_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool block_{false};
  bool stopped_{false};
  bool entered_{false};
  std::size_t wait_count_{0U};
  std::size_t stop_requests_{0U};
};

class StrategyHarness final
{
public:
  StrategyHarness()
  : backend(log), planning(
      backend, revision_hook, PlanningRetryPolicy{3U, 1ms, 1ms}, backoff),
    trajectory(log), gripper(log), scene(log), health(log),
    strategy(valid_config(), planning, trajectory, gripper, scene, health)
  {
    scene.reset();
    scene.refresh_robot_state = true;
    trajectory.on_success = [this]() {scene.advance_observed_robot_state();};
  }

  manipulation::task::TaskOutcome run(
    const bool return_home = false,
    const RecoveryPolicy recovery = RecoveryPolicy::kNone)
  {
    auto task = resolved_task(return_home, recovery);
    return strategy.execute(task, context);
  }

  EventLog log;
  FakePlanningBackend backend;
  CurrentRevisionHook revision_hook;
  ImmediateBackoff backoff;
  ArmPlanningPort planning;
  FakeTrajectoryPort trajectory;
  FakeGripperPort gripper;
  FakeScenePort scene;
  FakeHealthPort health;
  FakeContext context;
  PickLiftPlaceStrategy strategy;
};

bool has_prefix(const std::string & value, const std::string & prefix)
{
  return value.compare(0U, prefix.size(), prefix) == 0;
}

std::vector<std::string> externally_visible_steps(const EventLog & log)
{
  std::vector<std::string> result;
  for (const auto & event : log.snapshot()) {
    if (has_prefix(event, "plan:") || has_prefix(event, "trajectory:") ||
      has_prefix(event, "gripper:") || event == "scene:attach" ||
      event == "scene:detach")
    {
      result.push_back(event);
    }
  }
  return result;
}

std::size_t event_count(const EventLog & log, const std::string & value)
{
  const auto events = log.snapshot();
  return static_cast<std::size_t>(std::count(events.begin(), events.end(), value));
}

TEST(PickLiftPlacePortContract, AcceptedTimeoutAndSuccessSafetyAreEnforced)
{
  EXPECT_THROW(
    (void)PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kTimedOut,
      PhysicalCommandSafety::kNotAccepted,
      "impossible pre-acceptance timeout"),
    std::invalid_argument);

  const auto success = PhysicalCommandResult::success("done");
  EXPECT_TRUE(success.succeeded());
  EXPECT_EQ(success.safety(), PhysicalCommandSafety::kTerminalSafe);
  EXPECT_THROW(
    (void)PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kSuccess,
      PhysicalCommandSafety::kTerminalSafe,
      "not a failure"),
    std::invalid_argument);
}

TEST(PickLiftPlaceStrategyTest, ExecutesOrderedStagesWithFullSceneEvidenceAndHome)
{
  StrategyHarness harness;

  const auto outcome = harness.run(true);

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kObjectAtTarget);
  EXPECT_EQ(
    externally_visible_steps(harness.log),
    (std::vector<std::string>{
        "gripper:initial_open",
        "plan:pregrasp", "trajectory:pregrasp",
        "plan:approach_to_grasp", "trajectory:approach_to_grasp",
        "gripper:close", "scene:attach",
        "plan:lift", "trajectory:lift",
        "plan:preplace", "trajectory:preplace",
        "plan:place", "trajectory:place",
        "gripper:release", "scene:detach",
        "plan:retreat", "trajectory:retreat",
        "plan:home", "trajectory:home"}));
  ASSERT_FALSE(harness.backend.requests.empty());
  for (const auto & request : harness.backend.requests) {
    EXPECT_FALSE(request.scene_snapshot.is_diff);
    EXPECT_FALSE(request.scene_snapshot.robot_state.is_diff);
    EXPECT_EQ(request.scene_revision.session_id, "scene-session");
  }
  EXPECT_TRUE(harness.scene.request_evidence_consistent);
  EXPECT_TRUE(std::all_of(
      harness.trajectory.observed_timeouts.begin(),
      harness.trajectory.observed_timeouts.end(),
      [](const auto timeout) {return timeout == 3s;}));
  EXPECT_TRUE(std::all_of(
      harness.gripper.observed_timeouts.begin(),
      harness.gripper.observed_timeouts.end(),
      [](const auto timeout) {return timeout == 2s;}));
  const auto waits = harness.context.waiting_reasons();
  EXPECT_NE(std::find(waits.begin(), waits.end(), WaitingReason::kHealthReadiness), waits.end());
  EXPECT_NE(std::find(waits.begin(), waits.end(), WaitingReason::kSceneReadiness), waits.end());
  EXPECT_NE(std::find(waits.begin(), waits.end(), WaitingReason::kPlanning), waits.end());
  EXPECT_NE(std::find(waits.begin(), waits.end(), WaitingReason::kNone), waits.end());
}

TEST(PickLiftPlaceStrategyTest, ClearanceWaypointsFollowReversedApproachDirection)
{
  StrategyHarness harness;

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  const auto find_position = [&](const std::string & label) {
      const auto it = std::find_if(
        harness.backend.requests.begin(), harness.backend.requests.end(),
        [&](const ArmPlanningRequest & request) {return request.label == label;});
      EXPECT_NE(it, harness.backend.requests.end()) << label;
      return it->target_pose.pose.position;
    };
  // The fixture candidate approaches along +X, so every clearance waypoint
  // must depart along -X instead of the former hard-coded +Z lift.
  const auto grasp = find_position("approach_to_grasp");
  const auto lift = find_position("lift");
  EXPECT_NEAR(lift.x, grasp.x - 0.1, 1e-9);
  EXPECT_NEAR(lift.y, grasp.y, 1e-9);
  EXPECT_NEAR(lift.z, grasp.z, 1e-9);

  const auto place = find_position("place");
  const auto preplace = find_position("preplace");
  const auto retreat = find_position("retreat");
  EXPECT_NEAR(preplace.x, place.x - 0.12, 1e-9);
  EXPECT_NEAR(preplace.y, place.y, 1e-9);
  EXPECT_NEAR(preplace.z, place.z, 1e-9);
  EXPECT_NEAR(retreat.x, place.x - 0.15, 1e-9);
  EXPECT_NEAR(retreat.y, place.y, 1e-9);
  EXPECT_NEAR(retreat.z, place.z, 1e-9);
}

TEST(PickLiftPlaceStrategyTest, ForwardsCandidateCloseCommandAndKeepsOpenCommandsNeutral)
{
  StrategyHarness harness;

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  ASSERT_EQ(harness.gripper.requests.size(), 3U);

  const auto & initial_open = harness.gripper.requests.at(0);
  EXPECT_EQ(initial_open.kind, GripperCommandKind::kOpen);
  EXPECT_DOUBLE_EQ(initial_open.width_m, 0.08);
  EXPECT_DOUBLE_EQ(initial_open.max_effort_n, 0.0);
  EXPECT_FALSE(initial_open.allow_stall);
  EXPECT_EQ(initial_open.label, "initial_open");

  const auto & close = harness.gripper.requests.at(1);
  EXPECT_EQ(close.kind, GripperCommandKind::kClose);
  EXPECT_DOUBLE_EQ(close.width_m, 0.04);
  EXPECT_DOUBLE_EQ(close.max_effort_n, 20.0);
  EXPECT_TRUE(close.allow_stall);
  EXPECT_EQ(close.label, "close");

  const auto & release = harness.gripper.requests.at(2);
  EXPECT_EQ(release.kind, GripperCommandKind::kOpen);
  EXPECT_DOUBLE_EQ(release.width_m, 0.08);
  EXPECT_DOUBLE_EQ(release.max_effort_n, 0.0);
  EXPECT_FALSE(release.allow_stall);
  EXPECT_EQ(release.label, "release");
}

TEST(PickLiftPlaceStrategyTest, RefreshesRobotStateBeforeConstructingEveryPlan)
{
  StrategyHarness harness;
  harness.scene.refresh_robot_state = true;
  harness.trajectory.on_success = [&harness]() {
      harness.scene.advance_observed_robot_state();
    };

  const auto outcome = harness.run(true);

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  ASSERT_FALSE(harness.backend.requests.empty());
  double previous_position = -1.0;
  for (const auto & request : harness.backend.requests) {
    ASSERT_EQ(request.scene_snapshot.robot_state.joint_state.name.size(), 1U);
    ASSERT_EQ(request.scene_snapshot.robot_state.joint_state.position.size(), 1U);
    const double position = request.scene_snapshot.robot_state.joint_state.position.front();
    EXPECT_GT(position, previous_position);
    previous_position = position;
  }
}

TEST(PickLiftPlaceStrategyTest, WaitsForExecutedEndpointBeforeConstructingNextPlan)
{
  StrategyHarness harness;
  bool update_pending = false;
  int stale_reads_remaining = 0;
  int stale_reads = 0;
  harness.trajectory.on_success = [&]() {
      update_pending = true;
      stale_reads_remaining = 3;
    };
  harness.scene.on_revalidate = [&]() {
      if (!update_pending) {
        return;
      }
      ++stale_reads;
      --stale_reads_remaining;
      if (stale_reads_remaining == 0) {
        harness.scene.advance_observed_robot_state();
        update_pending = false;
      }
    };

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  ASSERT_GE(harness.backend.requests.size(), 2U);
  EXPECT_DOUBLE_EQ(
    harness.backend.requests.at(1).scene_snapshot.robot_state.joint_state.position.front(),
    0.1);
  EXPECT_GE(stale_reads, 3);
  const auto feedback = harness.context.feedback();
  EXPECT_TRUE(std::any_of(
    feedback.cbegin(), feedback.cend(), [](const auto & state) {
        return state.detail.find("last trajectory endpoint") != std::string::npos;
    }));
}

TEST(PickLiftPlaceStrategyTest, DiscardsPlanWhenRobotStateChangesBeforeDispatch)
{
  StrategyHarness harness;
  bool injected_change = false;
  harness.scene.on_revalidate = [&]() {
      if (!injected_change && harness.backend.requests.size() == 1U &&
        harness.trajectory.observed_timeouts.empty())
      {
        harness.scene.advance_observed_robot_state();
        injected_change = true;
      }
    };

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  ASSERT_TRUE(injected_change);
  ASSERT_GE(harness.backend.requests.size(), 2U);
  EXPECT_EQ(event_count(harness.log, "plan:pregrasp"), 2U);
  EXPECT_EQ(event_count(harness.log, "trajectory:pregrasp"), 1U);
  EXPECT_DOUBLE_EQ(
    harness.backend.requests.at(1).scene_snapshot.robot_state.joint_state.position.front(),
    0.1);
  const auto feedback = harness.context.feedback();
  EXPECT_TRUE(std::any_of(
      feedback.cbegin(), feedback.cend(), [](const auto & state) {
        return state.detail.find("discarding trajectory") != std::string::npos;
      }));
}

TEST(PickLiftPlaceStrategyTest, PreDispatchReplanAccumulatesBackendRetryEvidence)
{
  StrategyHarness harness;
  harness.backend.budget_failures_remaining = 1U;
  bool injected_change = false;
  harness.scene.on_revalidate = [&]() {
      if (!injected_change && harness.backend.requests.size() == 2U &&
        harness.trajectory.observed_timeouts.empty())
      {
        harness.scene.advance_observed_robot_state();
        injected_change = true;
      }
    };

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  EXPECT_EQ(event_count(harness.log, "plan:pregrasp"), 3U);
  EXPECT_EQ(event_count(harness.log, "trajectory:pregrasp"), 1U);
  const auto feedback = harness.context.feedback();
  EXPECT_NE(
    std::find_if(feedback.cbegin(), feedback.cend(), [](const auto & state) {
      return state.detail == "executing pregrasp" && state.retry_count == 2U;
    }),
    feedback.cend());
}

TEST(PickLiftPlaceStrategyTest, RepeatedPreDispatchStateChangesFailBeforeExecution)
{
  StrategyHarness harness;
  std::size_t observed_plans = 0U;
  harness.scene.on_revalidate = [&]() {
      if (harness.trajectory.observed_timeouts.empty() &&
        harness.backend.requests.size() > observed_plans)
      {
        observed_plans = harness.backend.requests.size();
        harness.scene.advance_observed_robot_state();
      }
    };

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kPlanningFailed);
  EXPECT_EQ(event_count(harness.log, "plan:pregrasp"), 3U);
  EXPECT_EQ(event_count(harness.log, "trajectory:pregrasp"), 0U);
  EXPECT_TRUE(harness.trajectory.observed_timeouts.empty());
}

TEST(PickLiftPlaceStrategyTest, CancelInterruptsPreDispatchStateReplan)
{
  StrategyHarness harness;
  bool injected_change = false;
  harness.scene.on_revalidate = [&]() {
      if (!injected_change && harness.backend.requests.size() == 1U &&
        harness.trajectory.observed_timeouts.empty())
      {
        harness.scene.advance_observed_robot_state();
        injected_change = true;
      }
    };
  harness.context.stop_on_wait();

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(event_count(harness.log, "plan:pregrasp"), 1U);
  EXPECT_TRUE(harness.trajectory.observed_timeouts.empty());
}

TEST(PickLiftPlaceStrategyTest, CancelInterruptsExecutedEndpointSynchronization)
{
  StrategyHarness harness;
  harness.trajectory.on_success = []() {};
  harness.context.stop_on_wait();

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(harness.backend.requests.size(), 1U);
  EXPECT_EQ(harness.trajectory.observed_timeouts.size(), 1U);
}

TEST(PickLiftPlaceStrategyTest, PauseResumesThroughFreshHealthAndSceneGates)
{
  StrategyHarness harness;
  harness.context.pause_on_checkpoint(1U);

  auto future = std::async(std::launch::async, [&harness]() {return harness.run();});
  ASSERT_TRUE(harness.context.wait_until_paused(1s));
  EXPECT_TRUE(harness.log.snapshot().empty());

  harness.context.resume();
  const auto outcome = future.get();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  const auto events = harness.log.snapshot();
  ASSERT_GE(events.size(), 3U);
  EXPECT_EQ(events[0], "health:wait");
  EXPECT_EQ(events[1], "scene:revalidate");
  EXPECT_EQ(events[2], "gripper:initial_open");
}

TEST(PickLiftPlaceStrategyTest, PlanningBudgetRetryStaysInsidePlanningPort)
{
  StrategyHarness harness;
  harness.backend.budget_failures_remaining = 1U;

  const auto outcome = harness.run();

  ASSERT_TRUE(outcome.succeeded()) << outcome.detail();
  EXPECT_EQ(event_count(harness.log, "plan:pregrasp"), 2U);
  const auto feedback = harness.context.feedback();
  EXPECT_NE(
    std::find_if(feedback.begin(), feedback.end(), [](const auto & state) {
      return state.detail == "executing pregrasp" && state.retry_count == 1U;
    }),
    feedback.end());
}

TEST(PickLiftPlaceStrategyTest, TerminalSafePhysicalTimeoutReleasesResources)
{
  StrategyHarness harness;
  harness.trajectory.failure_label = "pregrasp";
  harness.trajectory.failure = PhysicalCommandResult::failure(
    PhysicalCommandOutcome::kTimedOut,
    PhysicalCommandSafety::kTerminalSafe,
    "accepted trajectory stopped safely after timeout");

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kTimeout);
  EXPECT_EQ(
    outcome.resource_release(),
    manipulation::task::ResourceReleaseDisposition::kReleaseSafe);
}

TEST(PickLiftPlaceStrategyTest, UnknownPhysicalStateRequiresQuarantine)
{
  StrategyHarness harness;
  harness.trajectory.failure_label = "pregrasp";
  harness.trajectory.failure = PhysicalCommandResult::failure(
    PhysicalCommandOutcome::kTimedOut,
    PhysicalCommandSafety::kActiveStateUnknown,
    "accepted trajectory did not confirm stop");

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kTimeout);
  EXPECT_EQ(
    outcome.resource_release(),
    manipulation::task::ResourceReleaseDisposition::kQuarantineRequired);
}

TEST(PickLiftPlaceStrategyTest, CancelAfterRevalidationStartsNoPhysicalSideEffect)
{
  StrategyHarness harness;
  harness.scene.on_revalidate = [&harness]() {harness.context.cancel();};

  const auto outcome = harness.run();

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_TRUE(externally_visible_steps(harness.log).empty());
}

TEST(PickLiftPlaceStrategyTest, SceneRevisionMismatchFailsBeforePhysicalSideEffect)
{
  StrategyHarness harness;
  harness.scene.revision_mismatch = true;

  const auto outcome = harness.run();

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kSceneConflict);
  EXPECT_TRUE(externally_visible_steps(harness.log).empty());
}

TEST(PickLiftPlaceStrategyTest, CancelDoesNotMaskUncertainAttachMutation)
{
  StrategyHarness harness;
  harness.scene.attach_failure = CoordinatedSceneOutcome::kStateUncertain;
  harness.scene.on_attach = [&harness]() {harness.context.cancel();};

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(
    outcome.disposition(), manipulation::task::ObjectDisposition::kSceneUncertain);
}

TEST(PickLiftPlaceStrategyTest, CancelDoesNotMaskUncertainDetachMutation)
{
  StrategyHarness harness;
  harness.scene.detach_failure = CoordinatedSceneOutcome::kStateUncertain;
  harness.scene.on_detach = [&harness]() {harness.context.cancel();};

  const auto outcome = harness.run();

  EXPECT_FALSE(outcome.succeeded());
  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(
    outcome.disposition(), manipulation::task::ObjectDisposition::kSceneUncertain);
}

TEST(PickLiftPlaceStrategyTest, CancelAfterConfirmedClosePreservesHoldingAndSceneLease)
{
  StrategyHarness harness;
  harness.gripper.on_command = [&harness](const auto & request) {
      if (request.label == "close") {
        harness.context.cancel();
      }
    };

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kHoldingObject);
  EXPECT_EQ(
    manipulation::workflow::scene_lease_release_decision(outcome),
    manipulation::workflow::SceneLeaseReleaseDecision::kRetain);
  EXPECT_EQ(event_count(harness.log, "scene:attach"), 0U);
  EXPECT_EQ(outcome.recovery_count(), 0U);
}

TEST(PickLiftPlaceStrategyTest, CancelAfterConfirmedReleaseRetainsLeaseUntilDetach)
{
  StrategyHarness harness;
  harness.gripper.on_command = [&harness](const auto & request) {
      if (request.label == "release") {
        harness.context.cancel();
      }
    };

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kSceneUncertain);
  EXPECT_EQ(
    outcome.resource_release(),
    manipulation::task::ResourceReleaseDisposition::kReleaseSafe);
  EXPECT_NE(outcome.detail().find("release at target confirmed"), std::string::npos);
  EXPECT_EQ(
    manipulation::workflow::scene_lease_release_decision(outcome),
    manipulation::workflow::SceneLeaseReleaseDecision::kRetain);
  EXPECT_EQ(event_count(harness.log, "scene:detach"), 0U);
  EXPECT_EQ(outcome.recovery_count(), 0U);
}

TEST(PickLiftPlaceStrategyTest, CancelBeforeDetachPreservesSceneUncertainty)
{
  StrategyHarness harness;
  harness.gripper.on_command = [&harness](const auto & request) {
      if (request.label == "release") {
        harness.scene.on_revalidate = [&harness]() {harness.context.cancel();};
      }
    };

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kSceneUncertain);
  EXPECT_EQ(
    outcome.resource_release(),
    manipulation::task::ResourceReleaseDisposition::kReleaseSafe);
  EXPECT_EQ(
    manipulation::workflow::scene_lease_release_decision(outcome),
    manipulation::workflow::SceneLeaseReleaseDecision::kRetain);
  EXPECT_EQ(event_count(harness.log, "scene:detach"), 0U);
  EXPECT_EQ(outcome.recovery_count(), 0U);
}

TEST(PickLiftPlaceStrategyTest, CanceledGripperUnknownEvidenceRetainsSceneLease)
{
  for (const std::string label : {"close", "release"}) {
    SCOPED_TRACE(label);
    StrategyHarness harness;
    harness.gripper.failure_label = label;
    harness.gripper.failure = StrategyGripperCommandResult::failure(
      PhysicalCommandOutcome::kCanceled, PhysicalCommandSafety::kTerminalSafe,
      GripperPostcondition::kUnknown, "accepted gripper canceled with unknown contact");
    harness.gripper.on_command = [&harness, &label](const auto & request) {
        if (request.label == label) {
          harness.context.cancel();
        }
      };

    const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

    EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
    EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kUnknown);
    EXPECT_EQ(
      manipulation::workflow::scene_lease_release_decision(outcome),
      manipulation::workflow::SceneLeaseReleaseDecision::kRetain);
    EXPECT_EQ(outcome.recovery_count(), 0U);
  }
}

TEST(PickLiftPlaceStrategyTest, FailureAfterAttachRunsRetreatAndHomeRecovery)
{
  StrategyHarness harness;
  harness.trajectory.failure_label = "lift";
  harness.trajectory.failure = PhysicalCommandResult::failure(
    PhysicalCommandOutcome::kFailed,
    PhysicalCommandSafety::kTerminalSafe,
    "lift controller failed safely");

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kExecutionFailed);
  EXPECT_EQ(outcome.recovery_count(), 1U);
  EXPECT_EQ(outcome.disposition(), manipulation::task::ObjectDisposition::kHoldingObject);
  EXPECT_EQ(event_count(harness.log, "plan:recovery_retreat"), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_retreat"), 1U);
  EXPECT_EQ(event_count(harness.log, "plan:recovery_home"), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_home"), 1U);
}

TEST(PickLiftPlaceStrategyTest, CancelDuringRecoveryRetreatRemainsCanceled)
{
  StrategyHarness harness;
  harness.trajectory.failure_label = "lift";
  harness.trajectory.failure = PhysicalCommandResult::failure(
    PhysicalCommandOutcome::kFailed,
    PhysicalCommandSafety::kTerminalSafe,
    "lift controller failed safely");
  harness.trajectory.on_execute = [&harness](const auto & request) {
      if (request.label == "recovery_retreat") {
        harness.context.cancel();
      }
    };

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(outcome.recovery_count(), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_retreat"), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_home"), 0U);
}

TEST(PickLiftPlaceStrategyTest, CancelDuringRecoveryHomeRemainsCanceled)
{
  StrategyHarness harness;
  harness.trajectory.failure_label = "lift";
  harness.trajectory.failure = PhysicalCommandResult::failure(
    PhysicalCommandOutcome::kFailed,
    PhysicalCommandSafety::kTerminalSafe,
    "lift controller failed safely");
  harness.trajectory.on_execute = [&harness](const auto & request) {
      if (request.label == "recovery_home") {
        harness.context.cancel();
      }
    };

  const auto outcome = harness.run(false, RecoveryPolicy::kRetreatAndHome);

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(outcome.recovery_count(), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_retreat"), 1U);
  EXPECT_EQ(event_count(harness.log, "trajectory:recovery_home"), 1U);
}

TEST(PickLiftPlaceStrategyTest, LongHealthWaitIsCanceledWithoutSideEffects)
{
  StrategyHarness harness;
  harness.health.set_blocked(true);

  auto future = std::async(std::launch::async, [&harness]() {return harness.run();});
  ASSERT_TRUE(harness.health.wait_until_entered(1s));
  EXPECT_TRUE(externally_visible_steps(harness.log).empty());

  harness.context.cancel();
  harness.strategy.request_stop();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto outcome = future.get();

  EXPECT_EQ(outcome.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(
    outcome.resource_release(),
    manipulation::task::ResourceReleaseDisposition::kReleaseSafe);
  EXPECT_TRUE(externally_visible_steps(harness.log).empty());
  EXPECT_EQ(event_count(harness.log, "scene:revalidate"), 0U);
}

}  // namespace
}  // namespace manipulation::moveit
