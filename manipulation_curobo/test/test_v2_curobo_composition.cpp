// Responsibility: Prove the sole V2 workflow can plan through cuRobo and use common physical ports.

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gtest/gtest.h"

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/object_catalog.hpp"
#include "manipulation/workflow/resolved_transfer_workflow.hpp"
#include "manipulation/workflow/strategy_registry.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation/workflow/transfer_scene_workflow_port.hpp"
#include "manipulation/workflow/transfer_workflow_ports.hpp"
#include "manipulation/workflow/transfer_workflow_types.hpp"
#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"
#include "manipulation_moveit/arm_planning_port.hpp"
#include "manipulation_moveit/pick_lift_place_ports.hpp"
#include "manipulation_moveit/pick_lift_place_strategy.hpp"

#include "manipulation_curobo/temporary_file.hpp"

namespace
{

using namespace std::chrono_literals;
namespace workflow = manipulation::workflow;
namespace moveit_adapter = manipulation::moveit;
using manipulation_curobo::curobo::TemporaryFile;

class RosEnvironment final : public testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_context_ = true;
    }
  }

  void TearDown() override
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

[[maybe_unused]] const auto * const ros_environment =
  testing::AddGlobalTestEnvironment(new RosEnvironment);

void writeText(const TemporaryFile & file, const std::string & text)
{
  std::ofstream stream(file.path(), std::ios::out | std::ios::trunc);
  ASSERT_TRUE(stream.is_open());
  stream << text;
  ASSERT_TRUE(stream.good());
}

std::string fakeBridgeSource(const std::string & workspace_frame)
{
  return
    "import sys\n"
    "import yaml\n"
    "output = sys.argv[sys.argv.index('--output') + 1]\n"
    "if '--probe-config' in sys.argv:\n"
    "    with open(output, 'w', encoding='utf-8') as stream:\n"
    "        yaml.safe_dump({'success': True, 'workspace_frame': '" + workspace_frame +
    "', 'ee_link': 'panda_hand', 'tensor_device': 'fake:cpu'}, stream)\n"
    "    raise SystemExit(0)\n"
    "input_path = sys.argv[sys.argv.index('--input') + 1]\n"
    "with open(input_path, encoding='utf-8') as stream:\n"
    "    request = yaml.safe_load(stream)\n"
    "names = request['joint_names']\n"
    "start = request['start_positions']\n"
    "point = {'positions': start, 'velocities': [0.0] * len(names), "
    "'accelerations': [0.0] * len(names)}\n"
    "response = {'success': True, 'planning_time': 0.01, 'trajectory': "
    "{'dt': 0.01, 'joint_names': names, 'points': [point, point]}}\n"
    "with open(output, 'w', encoding='utf-8') as stream:\n"
    "    yaml.safe_dump(response, stream)\n";
}

std::shared_ptr<planning_interface::PlannerManager> makePlanner(
  const moveit::core::RobotModelPtr & model,
  const TemporaryFile & bridge,
  const TemporaryFile & config)
{
  static pluginlib::ClassLoader<planning_interface::PlannerManager> loader(
    "moveit_core", "planning_interface::PlannerManager");
  auto planner = loader.createSharedInstance("manipulation_curobo/CuRoboPlanner");
  auto node = std::make_shared<rclcpp::Node>("v2_curobo_composition_planner");
  node->declare_parameter("composition.python_executable", "/usr/bin/python3");
  node->declare_parameter("composition.bridge_script", bridge.path().string());
  node->declare_parameter("composition.motion_gen_config", config.path().string());
  node->declare_parameter("composition.ee_link", "panda_hand");
  if (!planner->initialize(model, node, "composition")) {
    return {};
  }
  return planner;
}

geometry_msgs::msg::Pose rigidPose(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::msg::PoseStamped stampedPose(
  const std::string & frame, const double x, const double y, const double z)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp.sec = 42;
  pose.header.stamp.nanosec = 123U;
  pose.pose = rigidPose(x, y, z);
  return pose;
}

class ImmediateContext final : public workflow::TransferExecutionContext
{
public:
  workflow::TransferExecutionControlResult checkpoint() override
  {
    return workflow::TransferExecutionControlResult::kContinue;
  }

  bool stop_requested() const noexcept override
  {
    return false;
  }

  workflow::TransferExecutionControlResult wait_until(TimePoint) override
  {
    return workflow::TransferExecutionControlResult::kContinue;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override
  {
  }
};

class ConfiguredObservation final : public workflow::ObservationProvider
{
public:
  explicit ConfiguredObservation(std::string frame)
  : frame_(std::move(frame))
  {
  }

  workflow::ObservationProviderResult observe(
    const workflow::ObjectInstance & instance,
    const workflow::ObjectProfile &,
    workflow::TransferExecutionContext &) override
  {
    ++calls;
    return workflow::ObservationProviderResult::success(
      workflow::ObjectObservation::create(
        instance.object_id(), stampedPose(frame_, 0.45, 0.0, 0.35),
        "configured-correlation", 1.0));
  }

  void request_stop() noexcept override
  {
  }

  std::size_t calls{0U};

private:
  std::string frame_;
};

class FixedGrasp final : public workflow::GraspProvider
{
public:
  workflow::GraspProviderResult generate(
    const workflow::ObjectObservation & observation,
    const workflow::ObjectProfile &,
    const workflow::PlanningSceneSnapshot &,
    workflow::TransferExecutionContext &) override
  {
    ++calls;
    auto pregrasp = observation.pose();
    pregrasp.pose.position.x -= 0.10;
    auto grasp = observation.pose();
    geometry_msgs::msg::Vector3Stamped approach;
    approach.header = observation.pose().header;
    approach.vector.x = 1.0;
    std::vector<workflow::GraspCandidate> candidates;
    candidates.emplace_back(workflow::GraspCandidate::create(
        observation.correlation(), std::move(pregrasp), std::move(grasp),
        std::move(approach), 0.04, 20.0, 1.0));
    return workflow::GraspProviderResult::success(std::move(candidates));
  }

  void request_stop() noexcept override
  {
  }

  std::size_t calls{0U};
};

class FullSceneState final
{
public:
  explicit FullSceneState(const moveit::core::RobotModelPtr & model)
  {
    const auto * group = model->getJointModelGroup("panda_arm");
    if (!group) {
      throw std::runtime_error("Panda planning group is unavailable");
    }
    const std::vector<double> positions{
      0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    planning_scene::PlanningScene scene(model);
    auto & state = scene.getCurrentStateNonConst();
    state.setToDefaultValues();
    state.setJointGroupPositions(group, positions);
    state.update();
    scene.getPlanningSceneMsg(scene_);
    scene_.is_diff = false;
    scene_.robot_state.is_diff = false;
  }

  workflow::PlanningSceneLease lease() const
  {
    return workflow::PlanningSceneLease::create(
      workflow::PlanningSceneSnapshot::create("composition-session", revision_, scene_),
      workflow::PlanningSceneAccessProof::create(
        "composition-lease", "composition-session", revision_));
  }

  workflow::PlanningSceneLease advance()
  {
    ++revision_;
    return lease();
  }

  std::uint64_t revision() const noexcept
  {
    return revision_;
  }

private:
  moveit_msgs::msg::PlanningScene scene_;
  std::uint64_t revision_{1U};
};

class WorkflowScenePort final : public workflow::TransferSceneWorkflowPort
{
public:
  explicit WorkflowScenePort(FullSceneState & state)
  : state_(state)
  {
  }

  workflow::PlanningSceneLeaseResult acquire(
    const std::vector<workflow::ObjectInstance> &,
    workflow::TransferExecutionContext &) override
  {
    return workflow::PlanningSceneLeaseResult::success(state_.lease());
  }

  workflow::PlanningSceneAccessProofResult renew(
    const workflow::PlanningSceneAccessProof & proof,
    workflow::TransferExecutionContext &) override
  {
    return workflow::PlanningSceneAccessProofResult::success(proof);
  }

  workflow::PlanningSceneLeaseResult stage_observation(
    const workflow::ObjectInstance &,
    const workflow::ObjectProfile &,
    const workflow::ObjectObservation &,
    const workflow::PlanningSceneAccessProof &,
    workflow::TransferExecutionContext &) override
  {
    return workflow::PlanningSceneLeaseResult::success(state_.advance());
  }

  workflow::PlanningSceneLeaseResult revalidate(
    const workflow::PlanningSceneAccessProof &,
    workflow::TransferExecutionContext &) override
  {
    return workflow::PlanningSceneLeaseResult::success(state_.lease());
  }

  void release(const workflow::PlanningSceneAccessProof &) noexcept override
  {
    ++release_count;
  }

  void request_stop() noexcept override
  {
  }

  std::size_t release_count{0U};

private:
  FullSceneState & state_;
};

class CurrentRevisionHook final : public moveit_adapter::IPlanningSceneRevisionHook
{
public:
  explicit CurrentRevisionHook(const FullSceneState & state)
  : state_(state)
  {
  }

  moveit_adapter::SceneRevisionCheck revalidate(
    const moveit_adapter::PlanningSceneRevision & expected,
    const moveit_adapter::PlanningStopToken & stop_token) override
  {
    if (stop_token.stop_requested()) {
      return {moveit_adapter::SceneRevisionState::kUnavailable, {}, "stopped"};
    }
    if (expected.session_id != "composition-session" ||
      expected.revision != state_.revision())
    {
      return {
        moveit_adapter::SceneRevisionState::kRefreshRequired,
        {"composition-session", state_.revision()}, "revision changed"};
    }
    return {moveit_adapter::SceneRevisionState::kCurrent, expected, "current"};
  }

private:
  const FullSceneState & state_;
};

class CuRoboBackendAdapter final : public moveit_adapter::IArmPlanningBackend
{
public:
  CuRoboBackendAdapter(
    moveit::core::RobotModelPtr model,
    std::shared_ptr<planning_interface::PlannerManager> planner)
  : model_(std::move(model)), planner_(std::move(planner))
  {
  }

  moveit_adapter::ArmPlanningReadiness cached_readiness() const override
  {
    return {planner_ != nullptr, planner_ ? "cuRobo fake runtime ready" : "unavailable"};
  }

  moveit_adapter::ArmPlanningResult plan_once(
    const moveit_adapter::ArmPlanningRequest & request,
    const moveit_adapter::PlanningStopToken & stop_token) override
  {
    if (stop_token.stop_requested()) {
      return stopped(stop_token);
    }
    auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
    if (!scene->setPlanningSceneMsg(request.scene_snapshot)) {
      return moveit_adapter::ArmPlanningResult::failure(
        moveit_adapter::PlanningOutcome::kInvalidRequest,
        "cannot reconstruct the immutable scene snapshot");
    }

    planning_interface::MotionPlanRequest moveit_request;
    moveit_request.group_name = "panda_arm";
    moveit_request.allowed_planning_time = 0.25;
    moveit_request.num_planning_attempts = 1;
    if (request.goal_kind == moveit_adapter::ArmPlanningGoalKind::kPose) {
      moveit_request.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(
          "panda_hand", request.target_pose, 0.001, 0.001));
    } else {
      moveit_msgs::msg::Constraints constraints;
      for (std::size_t index = 0U; index < request.joint_names.size(); ++index) {
        moveit_msgs::msg::JointConstraint joint;
        joint.joint_name = request.joint_names[index];
        joint.position = request.joint_positions[index];
        constraints.joint_constraints.push_back(std::move(joint));
      }
      moveit_request.goal_constraints.push_back(std::move(constraints));
    }

    moveit_msgs::msg::MoveItErrorCodes context_error;
    const auto context = planner_->getPlanningContext(
      scene, moveit_request, context_error);
    if (!context || context_error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      return moveit_adapter::ArmPlanningResult::failure(
        moveit_adapter::PlanningOutcome::kPlannerUnavailable,
        "cuRobo PlannerManager rejected the common planning request",
        context_error.val);
    }
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      active_context_ = context;
    }
    if (stop_token.stop_requested()) {
      (void)context->terminate();
    }
    planning_interface::MotionPlanResponse response;
    context->solve(response);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (active_context_ == context) {
        active_context_.reset();
      }
    }
    if (stop_token.stop_requested() ||
      response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED)
    {
      return stopped(stop_token);
    }
    if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
      !response.trajectory)
    {
      return moveit_adapter::ArmPlanningResult::failure(
        moveit_adapter::PlanningOutcome::kPlanningFailed,
        response.error_code.message.empty() ? "cuRobo planning failed" :
        response.error_code.message,
        response.error_code.val);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    response.trajectory->getRobotTrajectoryMsg(trajectory);
    ++plan_count;
    full_snapshot_count +=
      !request.scene_snapshot.is_diff && !request.scene_snapshot.robot_state.is_diff ? 1U : 0U;
    return moveit_adapter::ArmPlanningResult::success(
      std::move(trajectory), "planned by manipulation_curobo/CuRoboPlanner");
  }

  void request_stop() noexcept override
  {
    std::shared_ptr<planning_interface::PlanningContext> active;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      active = active_context_;
    }
    if (active) {
      (void)active->terminate();
    }
  }

  std::size_t plan_count{0U};
  std::size_t full_snapshot_count{0U};

private:
  static moveit_adapter::ArmPlanningResult stopped(
    const moveit_adapter::PlanningStopToken & stop_token)
  {
    return moveit_adapter::ArmPlanningResult::failure(
      stop_token.cause() == moveit_adapter::PlanningStopCause::kShutdown ?
      moveit_adapter::PlanningOutcome::kShutdown :
      moveit_adapter::PlanningOutcome::kCanceled,
      "cuRobo planning stopped");
  }

  moveit::core::RobotModelPtr model_;
  std::shared_ptr<planning_interface::PlannerManager> planner_;
  std::mutex mutex_;
  planning_interface::PlanningContextPtr active_context_;
};

class FakeTrajectoryPort final : public moveit_adapter::ITrajectoryExecutionPort
{
public:
  void begin_goal() override
  {
  }

  moveit_adapter::PhysicalCommandResult execute(
    const moveit_adapter::TrajectoryExecutionRequest & request,
    std::chrono::steady_clock::duration,
    workflow::TransferExecutionContext &) override
  {
    if (request.trajectory.joint_trajectory.points.empty()) {
      return moveit_adapter::PhysicalCommandResult::failure(
        moveit_adapter::PhysicalCommandOutcome::kFailed,
        moveit_adapter::PhysicalCommandSafety::kNotAccepted,
        "empty trajectory was not accepted");
    }
    ++execution_count;
    return moveit_adapter::PhysicalCommandResult::success("fake controller completed");
  }

  void request_stop() noexcept override
  {
  }

  std::size_t execution_count{0U};
};

class FakeGripperPort final : public moveit_adapter::IStrategyGripperCommandPort
{
public:
  void begin_goal() override
  {
  }

  moveit_adapter::StrategyGripperCommandResult command(
    const moveit_adapter::StrategyGripperCommandRequest & request,
    std::chrono::steady_clock::duration,
    workflow::TransferExecutionContext &) override
  {
    ++command_count;
    return moveit_adapter::StrategyGripperCommandResult::success(
      request.kind == moveit_adapter::GripperCommandKind::kOpen ?
      moveit_adapter::GripperPostcondition::kOpenConfirmed :
      moveit_adapter::GripperPostcondition::kClosedConfirmed,
      "fake gripper confirmed");
  }

  void request_stop() noexcept override
  {
  }

  std::size_t command_count{0U};
};

class FakeHealthPort final : public moveit_adapter::IPickLiftPlaceHealthPort
{
public:
  void begin_goal() override
  {
  }

  moveit_adapter::PickLiftPlaceHealthResult wait_until_ready(
    workflow::TransferExecutionContext &) override
  {
    return moveit_adapter::PickLiftPlaceHealthResult::ready("all fake physical ports ready");
  }

  void request_stop() noexcept override
  {
  }
};

class StrategyScenePort final : public moveit_adapter::IPickLiftPlaceScenePort
{
public:
  explicit StrategyScenePort(FullSceneState & state)
  : state_(state)
  {
  }

  void begin_goal() override
  {
  }

  moveit_adapter::CoordinatedSceneResult revalidate(
    const moveit_adapter::StrategySceneRevalidationRequest &,
    workflow::TransferExecutionContext &) override
  {
    return moveit_adapter::CoordinatedSceneResult::success(state_.lease(), "current");
  }

  moveit_adapter::CoordinatedSceneResult attach(
    const moveit_adapter::StrategySceneAttachRequest &,
    workflow::TransferExecutionContext &) override
  {
    ++attach_count;
    return moveit_adapter::CoordinatedSceneResult::success(state_.advance(), "attached");
  }

  moveit_adapter::CoordinatedSceneResult detach(
    const moveit_adapter::StrategySceneDetachRequest &,
    workflow::TransferExecutionContext &) override
  {
    ++detach_count;
    return moveit_adapter::CoordinatedSceneResult::success(state_.advance(), "detached");
  }

  void request_stop() noexcept override
  {
  }

  std::size_t attach_count{0U};
  std::size_t detach_count{0U};

private:
  FullSceneState & state_;
};

workflow::ObjectCatalog makeCatalog()
{
  auto profile = workflow::ObjectProfile::create(
    "cube_profile",
    workflow::ObjectGeometry::primitive(
      workflow::PrimitiveGeometry::create(
        workflow::PrimitiveGeometryType::kBox,
        {0.04, 0.04, 0.04}, rigidPose(0.0, 0.0, 0.0))));
  return workflow::ObjectCatalog::create(
    {std::move(profile)},
    {workflow::ObjectInstance::create("cube_1", "cube_profile")});
}

moveit_adapter::PickLiftPlaceStrategyConfig strategyConfig()
{
  moveit_adapter::PickLiftPlaceStrategyConfig config;
  config.robot.attachment_link = "panda_hand";
  config.robot.touch_links = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};
  config.robot.open_width_m = 0.08;
  config.robot.allow_grasp_stall = true;
  config.task.lift_clearance_m = 0.10;
  config.task.preplace_clearance_m = 0.10;
  config.task.retreat_clearance_m = 0.10;
  config.task.release_settle = 0ms;
  config.arm_motion_timeout = 1s;
  config.gripper_command_timeout = 1s;
  return config;
}

workflow::ResolvedTransferWorkflowOptions workflowOptions(const std::string & frame)
{
  workflow::ResolvedTransferWorkflowOptions options;
  options.validation.planning_frame = frame;
  options.validation.supported_recovery_policies = {
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_SAFE_STOP};
  return options;
}

workflow::ResolvedTransferWorkflow::Goal v2Goal(const std::string & frame)
{
  workflow::ResolvedTransferWorkflow::Goal goal;
  manipulation_msgs::msg::TransferObjectStepV2 step;
  step.object_id = "cube_1";
  step.target_mode = manipulation_msgs::msg::TransferObjectStepV2::TARGET_ABSOLUTE_POSE;
  step.target_object_pose = stampedPose(frame, 0.55, 0.10, 0.35);
  step.recovery_policy = manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_SAFE_STOP;
  step.return_home = false;
  goal.steps.emplace_back(std::move(step));
  return goal;
}

}  // namespace

TEST(V2CuRoboComposition, UsesTheCommonStrategyAndFakePhysicalPorts)
{
  const auto model = moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  TemporaryFile bridge{"v2_curobo_fake_bridge"};
  TemporaryFile config{"v2_curobo_fake_config"};
  writeText(bridge, fakeBridgeSource(model->getModelFrame()));
  writeText(config, "motion_gen: {}\n");
  const auto planner = makePlanner(model, bridge, config);
  ASSERT_NE(planner, nullptr);

  FullSceneState scene_state(model);
  CurrentRevisionHook revision_hook(scene_state);
  CuRoboBackendAdapter backend(model, planner);
  moveit_adapter::ArmPlanningPort planning(backend, revision_hook);
  FakeTrajectoryPort trajectory;
  FakeGripperPort gripper;
  FakeHealthPort health;
  StrategyScenePort strategy_scene(scene_state);
  auto strategy = std::make_unique<moveit_adapter::PickLiftPlaceStrategy>(
    strategyConfig(), planning, trajectory, gripper, strategy_scene, health);
  auto strategies = workflow::StrategyRegistry::production(std::move(strategy));

  const auto catalog = makeCatalog();
  ConfiguredObservation observation(model->getModelFrame());
  FixedGrasp grasp;
  WorkflowScenePort workflow_scene(scene_state);
  workflow::ResolvedTransferWorkflow workflow(
    workflowOptions(model->getModelFrame()), catalog, observation, grasp,
    workflow_scene, strategies, "pick_lift_place");
  ImmediateContext context;
  const auto goal = v2Goal(model->getModelFrame());

  ASSERT_TRUE(workflow.admit(goal).accepted);
  const auto execution = workflow.execute(goal, context, {}, "curobo-composition-goal");

  ASSERT_TRUE(execution.result.outcome.succeeded()) << execution.result.outcome.detail();
  EXPECT_EQ(execution.result.completed_step_count, 1U);
  EXPECT_EQ(observation.calls, 1U);
  EXPECT_EQ(grasp.calls, 1U);
  EXPECT_EQ(backend.plan_count, 6U);
  EXPECT_EQ(backend.full_snapshot_count, backend.plan_count);
  EXPECT_EQ(trajectory.execution_count, backend.plan_count);
  EXPECT_EQ(gripper.command_count, 3U);
  EXPECT_EQ(strategy_scene.attach_count, 1U);
  EXPECT_EQ(strategy_scene.detach_count, 1U);
  EXPECT_EQ(workflow_scene.release_count, 1U);
}
