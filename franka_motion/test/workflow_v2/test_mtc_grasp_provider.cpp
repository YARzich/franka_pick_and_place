// Responsibility: Verify MTC port correlation/ranking and local FixedState scene construction.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "moveit/task_constructor/stages/fixed_state.h"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "workflow_v2/mtc_grasp_provider.hpp"

namespace
{

using namespace std::chrono_literals;

class TestContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  using CheckpointHook = std::function<void()>;

  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    if (checkpoint_hook) {
      checkpoint_hook();
    }
    return stop_requested() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }
  bool stop_requested() const noexcept override
  {
    return stop_cause_value.load(std::memory_order_acquire) !=
           manipulation::workflow::TaskStopCause::kNone;
  }
  manipulation::workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return checkpoint();
  }
  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  manipulation::workflow::TaskStopCause stop_cause() const noexcept override
  {
    return stop_cause_value.load(std::memory_order_acquire);
  }

  void request_stop(const manipulation::workflow::TaskStopCause cause) noexcept
  {
    stop_cause_value.store(cause, std::memory_order_release);
  }

  CheckpointHook checkpoint_hook;

private:
  std::atomic<manipulation::workflow::TaskStopCause> stop_cause_value{
    manipulation::workflow::TaskStopCause::kNone};
};

geometry_msgs::msg::Pose rigid_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

manipulation::workflow::ObjectObservation observation_in_frame(std::string frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = std::move(frame_id);
  pose.header.stamp.sec = 7;
  pose.pose = rigid_pose();
  return manipulation::workflow::ObjectObservation::create("cube", pose, "cube@7.0");
}

manipulation::workflow::ObjectObservation observation()
{
  return observation_in_frame("base");
}

manipulation::workflow::ObjectProfile profile()
{
  return manipulation::workflow::ObjectProfile::create(
    "cube_profile",
    manipulation::workflow::ObjectGeometry::primitive(
      manipulation::workflow::PrimitiveGeometry::create(
        manipulation::workflow::PrimitiveGeometryType::kBox,
        {0.05, 0.05, 0.05}, rigid_pose())),
    "panda_top_down_cube");
}

manipulation::workflow::PlanningSceneSnapshot snapshot(
  geometry_msgs::msg::Pose object_pose = rigid_pose())
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = false;
  scene.name = "authoritative-copy";
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base";
  object.id = "cube";
  object.pose = std::move(object_pose);
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.05};
  object.primitives.push_back(std::move(box));
  object.primitive_poses.push_back(rigid_pose());
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.world.collision_objects.push_back(std::move(object));
  return manipulation::workflow::PlanningSceneSnapshot::create("session-1", 23U, scene);
}

manipulation::workflow::GraspCandidate candidate(
  const manipulation::workflow::ObjectObservation & source, const double score)
{
  auto grasp = source.pose();
  auto pregrasp = grasp;
  pregrasp.pose.position.z += 0.1;
  geometry_msgs::msg::Vector3Stamped approach;
  approach.header = source.pose().header;
  approach.vector.z = -1.0;
  return manipulation::workflow::GraspCandidate::create(
    source.correlation(), pregrasp, grasp, approach, 0.08, 20.0, score);
}

class FakeSnapshotPlanner final : public franka_motion::workflow_v2::MtcSnapshotPlanner
{
public:
  manipulation::workflow::GraspProviderResult plan(
    const manipulation::workflow::ObjectObservation & source,
    const manipulation::workflow::ObjectProfile &,
    const manipulation::workflow::PlanningSceneSnapshot & scene,
    const franka_motion::workflow_v2::MtcGraspProfileConfig &,
    manipulation::workflow::TransferExecutionContext &) override
  {
    seen_revision = scene.revision();
    seen_scene_name = scene.scene().name;
    return manipulation::workflow::GraspProviderResult::success(
      {candidate(source, 0.2), candidate(source, 0.9)});
  }

  void request_stop() noexcept override {++stop_calls;}

  std::uint64_t seen_revision{0U};
  std::string seen_scene_name;
  std::atomic<int> stop_calls{0};
};

class BlockingPlanOperation
{
public:
  bool plan()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [this]() {return completed_ || preempted_;});
    return completed_ && !preempted_;
  }

  void complete_success()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      completed_ = true;
    }
    changed_.notify_all();
  }

  void preempt()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      preempted_ = true;
    }
    preempt_calls_.fetch_add(1, std::memory_order_acq_rel);
    changed_.notify_all();
  }

  int preempt_calls() const noexcept
  {
    return preempt_calls_.load(std::memory_order_acquire);
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  bool completed_{false};
  bool preempted_{false};
  std::atomic<int> preempt_calls_{0};
};

franka_motion::workflow_v2::detail::FixedStateMtcTaskOperations fake_task_operations(
  BlockingPlanOperation & operation,
  std::atomic<int> & initialize_calls,
  std::atomic<int> & extract_calls)
{
  franka_motion::workflow_v2::detail::FixedStateMtcTaskOperations operations;
  operations.initialize = [&initialize_calls](moveit::task_constructor::Task &) {
      initialize_calls.fetch_add(1, std::memory_order_acq_rel);
    };
  operations.plan = [&operation](moveit::task_constructor::Task &, const std::size_t) {
      return operation.plan();
    };
  operations.preempt = [&operation](moveit::task_constructor::Task &) {
      operation.preempt();
    };
  operations.extract_candidates = [&extract_calls](
    moveit::task_constructor::Task &,
    const manipulation::workflow::ObjectObservation & source,
    const franka_motion::workflow_v2::MtcGraspProfileConfig &,
    const std::string &)
    {
      extract_calls.fetch_add(1, std::memory_order_acq_rel);
      return std::vector<manipulation::workflow::GraspCandidate>{candidate(source, 0.9)};
    };
  return operations;
}

franka_motion::workflow_v2::MtcGraspProfileConfig valid_config()
{
  franka_motion::workflow_v2::MtcGraspProfileConfig config;
  config.arm_group_name = "arm";
  config.eef_name = "hand";
  config.ik_frame_link = "tool";
  config.pregrasp_pose = "open";
  config.grasp_pose = "closed";
  config.angle_delta = 0.5;
  config.min_solution_distance = 0.1;
  config.max_ik_solutions = 4U;
  config.max_solutions = 2U;
  config.pregrasp_distance_m = 0.1;
  config.gripper_opening_m = 0.08;
  config.gripper_force_n = 20.0;
  return config;
}

class MtcFixedStateRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

moveit::core::RobotModelConstPtr test_robot_model(const std::string & node_name)
{
  auto node = std::make_shared<rclcpp::Node>(node_name);
  node->declare_parameter<std::string>(
    "robot_description", "<robot name='test'><link name='base'/></robot>");
  node->declare_parameter<std::string>(
    "robot_description_semantic", "<robot name='test'/>");
  moveit::task_constructor::Task model_task("model");
  model_task.loadRobotModel(node);
  return model_task.getRobotModel();
}

moveit::core::RobotModelConstPtr fixed_root_robot_model(const std::string & node_name)
{
  auto node = std::make_shared<rclcpp::Node>(node_name);
  node->declare_parameter<std::string>(
    "robot_description",
    "<robot name='test'>"
    "<link name='world'/><link name='base'/>"
    "<joint name='world_to_base' type='fixed'>"
    "<parent link='world'/><child link='base'/><origin xyz='0 0 1'/>"
    "</joint></robot>");
  node->declare_parameter<std::string>(
    "robot_description_semantic", "<robot name='test'/>");
  moveit::task_constructor::Task model_task("model");
  model_task.loadRobotModel(node);
  return model_task.getRobotModel();
}

franka_motion::workflow_v2::detail::FixedStateMtcTaskOperations successful_task_operations()
{
  franka_motion::workflow_v2::detail::FixedStateMtcTaskOperations operations;
  operations.initialize = [](moveit::task_constructor::Task &) {};
  operations.plan = [](moveit::task_constructor::Task &, const std::size_t) {return true;};
  operations.preempt = [](moveit::task_constructor::Task &) {};
  operations.extract_candidates = [](
    moveit::task_constructor::Task &,
    const manipulation::workflow::ObjectObservation & source,
    const franka_motion::workflow_v2::MtcGraspProfileConfig &,
    const std::string &)
    {
      return std::vector<manipulation::workflow::GraspCandidate>{candidate(source, 0.9)};
    };
  return operations;
}

void expect_blocking_plan_stop(
  const manipulation::workflow::TaskStopCause expected_cause,
  const moveit::core::RobotModelConstPtr & robot_model)
{
  BlockingPlanOperation operation;
  std::atomic<int> initialize_calls{0};
  std::atomic<int> extract_calls{0};
  franka_motion::workflow_v2::FixedStateMtcSnapshotPlanner planner(
    robot_model, "base",
    fake_task_operations(operation, initialize_calls, extract_calls));
  TestContext context;
  std::atomic<int> polls{0};
  std::atomic<int> downstream_side_effects{0};
  context.checkpoint_hook = [&context, &polls, expected_cause]() {
      if (polls.fetch_add(1, std::memory_order_acq_rel) + 1 >= 2) {
        context.request_stop(expected_cause);
      }
    };

  const auto started = std::chrono::steady_clock::now();
  const auto result = planner.plan(
    observation(), profile(), snapshot(), valid_config(), context);
  if (result.succeeded()) {
    downstream_side_effects.fetch_add(1, std::memory_order_acq_rel);
  }
  const auto wall_elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(context.stop_cause(), expected_cause);
  EXPECT_EQ(initialize_calls.load(std::memory_order_acquire), 1);
  EXPECT_EQ(extract_calls.load(std::memory_order_acquire), 0);
  EXPECT_EQ(operation.preempt_calls(), 1);
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 0);
  EXPECT_LT(wall_elapsed, 1s);

  operation.complete_success();
  EXPECT_EQ(extract_calls.load(std::memory_order_acquire), 0);
  EXPECT_EQ(operation.preempt_calls(), 1);
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 0);
}

}  // namespace

TEST(V2MtcGraspProvider, RanksCorrelatedCandidatesFromExactImmutableRevision)
{
  auto planner = std::make_unique<FakeSnapshotPlanner>();
  auto * planner_observer = planner.get();
  franka_motion::workflow_v2::MtcGraspProfiles profiles;
  profiles.emplace("panda_top_down_cube", valid_config());
  franka_motion::workflow_v2::MtcGraspProvider provider(
    std::move(profiles), std::move(planner));
  TestContext context;
  const auto source = observation();
  const auto scene = snapshot();

  const auto result = provider.generate(source, profile(), scene, context);

  ASSERT_TRUE(result.succeeded());
  ASSERT_EQ(result.value()->size(), 2U);
  EXPECT_DOUBLE_EQ(result.value()->front().score(), 0.9);
  EXPECT_EQ(result.value()->front().correlation(), source.correlation());
  EXPECT_EQ(planner_observer->seen_revision, 23U);
  EXPECT_EQ(planner_observer->seen_scene_name, "authoritative-copy");
  EXPECT_EQ(scene.scene().name, "authoritative-copy");

  provider.request_stop();
  const auto next_goal = provider.generate(source, profile(), scene, context);
  EXPECT_TRUE(next_goal.succeeded());
}

TEST_F(
  MtcFixedStateRosTest,
  ProductionBlockingPlanContinuesAfterVirtualWaitBeyondRemovedThirtySecondLimit)
{
  BlockingPlanOperation operation;
  std::atomic<int> initialize_calls{0};
  std::atomic<int> extract_calls{0};
  franka_motion::workflow_v2::FixedStateMtcSnapshotPlanner planner(
    test_robot_model("test_v2_mtc_delayed_success"),
    "base",
    fake_task_operations(operation, initialize_calls, extract_calls));
  TestContext context;
  std::atomic<int> virtual_elapsed_seconds{0};
  std::atomic<int> downstream_side_effects{0};
  context.checkpoint_hook = [&operation, &virtual_elapsed_seconds]() {
      constexpr int kVirtualPollSeconds = 5;
      const int elapsed =
        virtual_elapsed_seconds.fetch_add(kVirtualPollSeconds, std::memory_order_acq_rel) +
        kVirtualPollSeconds;
      if (elapsed > 30) {
        operation.complete_success();
      }
    };

  const auto started = std::chrono::steady_clock::now();
  const auto result = planner.plan(
    observation(), profile(), snapshot(), valid_config(), context);
  if (result.succeeded()) {
    downstream_side_effects.fetch_add(1, std::memory_order_acq_rel);
  }
  const auto wall_elapsed = std::chrono::steady_clock::now() - started;

  ASSERT_TRUE(result.succeeded());
  EXPECT_GT(virtual_elapsed_seconds.load(std::memory_order_acquire), 30);
  EXPECT_EQ(initialize_calls.load(std::memory_order_acquire), 1);
  EXPECT_EQ(extract_calls.load(std::memory_order_acquire), 1);
  EXPECT_EQ(operation.preempt_calls(), 0);
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 1);
  EXPECT_LT(wall_elapsed, 1s);
}

TEST_F(MtcFixedStateRosTest, AcceptsWorkflowFrameBelowDistinctFixedRootModelFrame)
{
  franka_motion::workflow_v2::FixedStateMtcSnapshotPlanner planner(
    fixed_root_robot_model("test_v2_mtc_distinct_workflow_frame"),
    "base",
    successful_task_operations());
  TestContext context;

  const auto result = planner.plan(
    observation(), profile(), snapshot(), valid_config(), context);

  ASSERT_TRUE(result.succeeded()) << result.detail();
  ASSERT_EQ(result.value()->size(), 1U);
  EXPECT_EQ(result.value()->front().grasp_pose().header.frame_id, "base");
}

TEST_F(MtcFixedStateRosTest, UnknownWorkflowFrameFailsBeforeAnyMtcTaskSideEffect)
{
  std::atomic<int> initialize_calls{0};
  std::atomic<int> plan_calls{0};
  std::atomic<int> preempt_calls{0};
  std::atomic<int> extract_calls{0};
  franka_motion::workflow_v2::detail::FixedStateMtcTaskOperations operations;
  operations.initialize = [&initialize_calls](moveit::task_constructor::Task &) {
      initialize_calls.fetch_add(1, std::memory_order_acq_rel);
    };
  operations.plan = [&plan_calls](moveit::task_constructor::Task &, const std::size_t) {
      plan_calls.fetch_add(1, std::memory_order_acq_rel);
      return true;
    };
  operations.preempt = [&preempt_calls](moveit::task_constructor::Task &) {
      preempt_calls.fetch_add(1, std::memory_order_acq_rel);
    };
  operations.extract_candidates = [&extract_calls](
    moveit::task_constructor::Task &,
    const manipulation::workflow::ObjectObservation & source,
    const franka_motion::workflow_v2::MtcGraspProfileConfig &,
    const std::string &)
    {
      extract_calls.fetch_add(1, std::memory_order_acq_rel);
      return std::vector<manipulation::workflow::GraspCandidate>{candidate(source, 0.9)};
    };
  franka_motion::workflow_v2::FixedStateMtcSnapshotPlanner planner(
    test_robot_model("test_v2_mtc_unknown_workflow_frame"),
    "base",
    std::move(operations));
  TestContext context;

  const auto result = planner.plan(
    observation_in_frame("unknown_observation_frame"),
    profile(), snapshot(), valid_config(), context);

  EXPECT_EQ(result.error_code(), manipulation::task::TaskErrorCode::kTfError);
  EXPECT_NE(result.detail().find("unknown_observation_frame"), std::string::npos);
  EXPECT_EQ(initialize_calls.load(std::memory_order_acquire), 0);
  EXPECT_EQ(plan_calls.load(std::memory_order_acquire), 0);
  EXPECT_EQ(preempt_calls.load(std::memory_order_acquire), 0);
  EXPECT_EQ(extract_calls.load(std::memory_order_acquire), 0);
}

TEST_F(MtcFixedStateRosTest, RendersInitFailureBeforeTaskOwnedStageLifetimeEnds)
{
  franka_motion::workflow_v2::FixedStateMtcSnapshotPlanner planner(
    test_robot_model("test_v2_mtc_init_failure_lifetime"), "base");
  TestContext context;

  const auto result = planner.plan(
    observation(), profile(), snapshot(), valid_config(), context);

  EXPECT_EQ(result.error_code(), manipulation::task::TaskErrorCode::kPlanningFailed);
  EXPECT_FALSE(result.detail().empty());
  EXPECT_EQ(result.detail().find('\0'), std::string::npos);
  EXPECT_EQ(result.detail().find("mtc::InitStageException"), std::string::npos);
}

TEST(V2MtcFrameConversion, ExpressesModelPoseInWorkflowFrameWithoutRelabeling)
{
  Eigen::Isometry3d model_to_workflow = Eigen::Isometry3d::Identity();
  model_to_workflow.translation() = Eigen::Vector3d{0.0, 0.0, 1.0};
  Eigen::Isometry3d model_to_link = Eigen::Isometry3d::Identity();
  model_to_link.translation() = Eigen::Vector3d{0.4, -0.2, 1.6};
  Eigen::Isometry3d link_to_pose = Eigen::Isometry3d::Identity();
  link_to_pose.translation() = Eigen::Vector3d{0.0, 0.0, 0.1};
  std_msgs::msg::Header header;
  header.frame_id = "base";
  header.stamp.sec = 7;

  const auto pose = franka_motion::workflow_v2::detail::express_link_pose_in_frame(
    model_to_workflow, model_to_link, link_to_pose, header);

  EXPECT_EQ(pose.header, header);
  EXPECT_NEAR(pose.pose.position.x, 0.4, 1.0e-12);
  EXPECT_NEAR(pose.pose.position.y, -0.2, 1.0e-12);
  EXPECT_NEAR(pose.pose.position.z, 0.7, 1.0e-12);
  EXPECT_NEAR(pose.pose.orientation.w, 1.0, 1.0e-12);
}

TEST(V2MtcFrameConversion, PreservesTransformOrderWithNonCommutingRotations)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  constexpr double kSqrtHalf = 0.70710678118654752440;
  Eigen::Isometry3d model_to_workflow = Eigen::Isometry3d::Identity();
  model_to_workflow.linear() =
    Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  model_to_workflow.translation() = Eigen::Vector3d{1.0, 2.0, 3.0};
  Eigen::Isometry3d model_to_link = Eigen::Isometry3d::Identity();
  model_to_link.linear() =
    Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  model_to_link.translation() = Eigen::Vector3d{4.0, 6.0, 8.0};
  Eigen::Isometry3d link_to_pose = Eigen::Isometry3d::Identity();
  link_to_pose.linear() =
    Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitY()).toRotationMatrix();
  link_to_pose.translation() = Eigen::Vector3d{1.0, 0.0, 0.0};
  std_msgs::msg::Header header;
  header.frame_id = "base";
  header.stamp.sec = 7;

  const auto pose = franka_motion::workflow_v2::detail::express_link_pose_in_frame(
    model_to_workflow, model_to_link, link_to_pose, header);

  EXPECT_EQ(pose.header, header);
  EXPECT_NEAR(pose.pose.position.x, 4.0, 1.0e-12);
  EXPECT_NEAR(pose.pose.position.y, -4.0, 1.0e-12);
  EXPECT_NEAR(pose.pose.position.z, 5.0, 1.0e-12);
  EXPECT_NEAR(pose.pose.orientation.x, kSqrtHalf, 1.0e-12);
  EXPECT_NEAR(pose.pose.orientation.y, 0.0, 1.0e-12);
  EXPECT_NEAR(pose.pose.orientation.z, 0.0, 1.0e-12);
  EXPECT_NEAR(pose.pose.orientation.w, kSqrtHalf, 1.0e-12);
}

TEST_F(
  MtcFixedStateRosTest,
  ProductionBlockingPlanClientCancelInterruptsQuicklyWithoutLaterSideEffect)
{
  expect_blocking_plan_stop(
    manipulation::workflow::TaskStopCause::kClientCanceled,
    test_robot_model("test_v2_mtc_client_cancel"));
}

TEST_F(
  MtcFixedStateRosTest,
  ProductionBlockingPlanShutdownInterruptsQuicklyWithoutLaterSideEffect)
{
  expect_blocking_plan_stop(
    manipulation::workflow::TaskStopCause::kShutdown,
    test_robot_model("test_v2_mtc_shutdown"));
}

TEST_F(MtcFixedStateRosTest, BuildsLocalSceneAndUsesFixedStateInsteadOfCurrentState)
{
  auto node = std::make_shared<rclcpp::Node>("test_v2_fixed_state_mtc");
  node->declare_parameter<std::string>(
    "robot_description", "<robot name='test'><link name='base'/></robot>");
  node->declare_parameter<std::string>(
    "robot_description_semantic", "<robot name='test'/>");
  moveit::task_constructor::Task model_task("model");
  model_task.loadRobotModel(node);

  const auto scene_snapshot = snapshot();
  auto local_scene = franka_motion::workflow_v2::detail::make_local_snapshot_scene(
    model_task.getRobotModel(), scene_snapshot);
  const auto grasp_reference_frame =
    franka_motion::workflow_v2::detail::add_workflow_aligned_grasp_frame(
    local_scene, "cube", "base");
  auto task = franka_motion::workflow_v2::detail::make_fixed_state_grasp_task(
    local_scene, grasp_reference_frame, valid_config());

  ASSERT_NE(local_scene, nullptr);
  EXPECT_EQ(local_scene->getName(), "authoritative-copy");
  const auto * initial = task.findChild("snapshot");
  ASSERT_NE(initial, nullptr);
  EXPECT_NE(
    dynamic_cast<const moveit::task_constructor::stages::FixedState *>(initial), nullptr);
  EXPECT_EQ(task.findChild("current"), nullptr);
  EXPECT_EQ(scene_snapshot.scene().name, "authoritative-copy");
}

TEST_F(MtcFixedStateRosTest, KeepsObservedOrientationButSamplesInWorkflowOrientation)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  geometry_msgs::msg::Pose object_pose;
  object_pose.position.x = 0.4;
  object_pose.position.y = -0.2;
  object_pose.position.z = 0.03;
  const Eigen::Quaterniond object_rotation(
    Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(kHalfPi, Eigen::Vector3d::UnitZ()));
  object_pose.orientation.x = object_rotation.x();
  object_pose.orientation.y = object_rotation.y();
  object_pose.orientation.z = object_rotation.z();
  object_pose.orientation.w = object_rotation.w();

  auto local_scene = franka_motion::workflow_v2::detail::make_local_snapshot_scene(
    test_robot_model("test_v2_mtc_workflow_aligned_grasp_frame"),
    snapshot(object_pose));
  const Eigen::Isometry3d object_before = local_scene->getFrameTransform("cube");
  const auto reference_frame =
    franka_motion::workflow_v2::detail::add_workflow_aligned_grasp_frame(
    local_scene, "cube", "base");
  const Eigen::Isometry3d object_after = local_scene->getFrameTransform("cube");
  const Eigen::Isometry3d reference = local_scene->getFrameTransform(reference_frame);
  const Eigen::Isometry3d workflow = local_scene->getFrameTransform("base");

  EXPECT_TRUE(object_before.matrix().isApprox(object_after.matrix(), 1.0e-12));
  EXPECT_TRUE(reference.translation().isApprox(object_before.translation(), 1.0e-12));
  EXPECT_TRUE(reference.linear().isApprox(workflow.linear(), 1.0e-12));
  EXPECT_FALSE(reference.linear().isApprox(object_before.linear(), 1.0e-12));
}
