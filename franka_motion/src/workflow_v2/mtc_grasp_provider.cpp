// Responsibility: Plans ranked correlated grasps on a local FixedState scene copy.

#include "workflow_v2/mtc_grasp_provider.hpp"

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "moveit/task_constructor/stage.h"
#include "moveit/task_constructor/stages/compute_ik.h"
#include "moveit/task_constructor/stages/fixed_state.h"
#include "moveit/task_constructor/stages/generate_grasp_pose.h"

#include "moveit/collision_detection/world.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "mtc/geometry_conversion.hpp"
#include "mtc/mtc_stop_monitor.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

namespace mtc = moveit::task_constructor;
using manipulation::task::TaskErrorCode;
using manipulation::workflow::GraspCandidate;
using manipulation::workflow::GraspProviderResult;

/** Join one immutable snapshot seed to IK candidates without planning motion. */
class GraspCandidateConnector final : public mtc::Connecting
{
public:
  GraspCandidateConnector()
  : mtc::Connecting("join snapshot to grasp candidate")
  {
  }

  void compute(
    const mtc::InterfaceState & from,
    const mtc::InterfaceState & to) override
  {
    mtc::SubTrajectory trajectory;
    trajectory.setCost(0.0);
    connect(from, to, std::move(trajectory));
  }

protected:
  bool compatible(
    const mtc::InterfaceState & from,
    const mtc::InterfaceState & to) const override
  {
    return from.scene() && to.scene() &&
           from.scene()->getRobotModel().get() == to.scene()->getRobotModel().get();
  }
};

bool is_valid_config(const MtcGraspProfileConfig & config)
{
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  return !config.arm_group_name.empty() && !config.eef_name.empty() &&
         !config.ik_frame_link.empty() && !config.pregrasp_pose.empty() &&
         !config.grasp_pose.empty() && config.ik_frame_transform.matrix().allFinite() &&
         config.rotation_axis.allFinite() &&
         std::abs(config.rotation_axis.norm() - 1.0) <= 1.0e-9 &&
         config.approach_direction.allFinite() &&
         std::abs(config.approach_direction.norm() - 1.0) <= 1.0e-9 &&
         std::isfinite(config.angle_delta) && config.angle_delta > 0.0 &&
         finite_non_negative(config.min_solution_distance) &&
         config.max_ik_solutions > 0U && config.max_solutions > 0U &&
         finite_non_negative(config.pregrasp_distance_m) &&
         finite_non_negative(config.gripper_opening_m) &&
         finite_non_negative(config.gripper_force_n);
}

std::string init_stage_error_detail(const mtc::InitStageException & error)
{
  std::ostringstream stream;
  stream << error;
  const auto raw = stream.str();
  std::string result;
  result.reserve(raw.size());
  for (const char character : raw) {
    if (character == '\0') {
      result += "\\0";
    } else {
      result += character;
    }
  }
  return result;
}

geometry_msgs::msg::PoseStamped pose_from_solution(
  const mtc::SolutionBase & solution,
  const manipulation::workflow::ObjectObservation & observation,
  const std::string & output_link_frame)
{
  const auto * state = solution.end() != nullptr ? solution.end() : solution.start();
  if (!state || !state->scene()) {
    throw std::runtime_error("MTC solution has no PlanningScene state");
  }
  const auto & scene = *state->scene();
  const auto & robot_state = scene.getCurrentState();
  if (!scene.knowsFrameTransform(robot_state, observation.pose().header.frame_id)) {
    throw std::runtime_error("MTC solution cannot resolve the workflow planning frame");
  }
  if (!scene.knowsFrameTransform(robot_state, output_link_frame)) {
    throw std::runtime_error("MTC solution does not contain the execution output link");
  }
  const Eigen::Isometry3d model_to_output_frame = scene.getFrameTransform(
    robot_state, observation.pose().header.frame_id);
  const Eigen::Isometry3d model_to_link_frame = scene.getFrameTransform(
    robot_state, output_link_frame);
  return detail::express_link_pose_in_frame(
    model_to_output_frame, model_to_link_frame, Eigen::Isometry3d::Identity(),
    observation.pose().header);
}

GraspCandidate candidate_from_solution(
  const mtc::SolutionBase & solution,
  const manipulation::workflow::ObjectObservation & observation,
  const MtcGraspProfileConfig & config,
  const std::string & output_link_frame)
{
  auto grasp = pose_from_solution(solution, observation, output_link_frame);
  auto pregrasp = grasp;
  pregrasp.pose.position.x -= config.approach_direction.x() * config.pregrasp_distance_m;
  pregrasp.pose.position.y -= config.approach_direction.y() * config.pregrasp_distance_m;
  pregrasp.pose.position.z -= config.approach_direction.z() * config.pregrasp_distance_m;

  geometry_msgs::msg::Vector3Stamped approach;
  approach.header = observation.pose().header;
  approach.vector.x = config.approach_direction.x();
  approach.vector.y = config.approach_direction.y();
  approach.vector.z = config.approach_direction.z();
  return GraspCandidate::create(
    observation.correlation(), std::move(pregrasp), std::move(grasp), std::move(approach),
    config.gripper_opening_m, config.gripper_force_n, -solution.cost());
}

}  // namespace

MtcGraspProvider::MtcGraspProvider(
  MtcGraspProfiles profiles,
  std::unique_ptr<MtcSnapshotPlanner> planner)
: profiles_(std::move(profiles)), planner_(std::move(planner))
{
  if (profiles_.empty() || !planner_) {
    throw std::invalid_argument("MTC grasp provider requires profiles and a planner");
  }
  for (const auto & [id, profile] : profiles_) {
    if (id.empty() || !is_valid_config(profile)) {
      throw std::invalid_argument("MTC grasp provider profile is invalid");
    }
  }
}

GraspProviderResult MtcGraspProvider::generate(
  const manipulation::workflow::ObjectObservation & observation,
  const manipulation::workflow::ObjectProfile & profile,
  const manipulation::workflow::PlanningSceneSnapshot & scene_snapshot,
  manipulation::workflow::TransferExecutionContext & context)
{
  const auto request_generation = stop_generation_.load(std::memory_order_acquire);
  if (context.stop_requested()) {
    return GraspProviderResult::failure(TaskErrorCode::kCanceled, "grasp generation was canceled");
  }
  if (!profile.grasp_profile_id()) {
    return GraspProviderResult::failure(
      TaskErrorCode::kInvalidGoal, "object profile has no grasp_profile_id");
  }
  const auto found = profiles_.find(*profile.grasp_profile_id());
  if (found == profiles_.cend()) {
    return GraspProviderResult::failure(
      TaskErrorCode::kInvalidGoal,
      "unknown MTC grasp profile '" + *profile.grasp_profile_id() + "'");
  }
  auto result = planner_->plan(observation, profile, scene_snapshot, found->second, context);
  if (stop_generation_.load(std::memory_order_acquire) != request_generation) {
    return GraspProviderResult::failure(TaskErrorCode::kCanceled, "grasp generation was canceled");
  }
  if (!result.succeeded()) {
    return result;
  }
  try {
    auto candidates = result.value().value();
    return GraspProviderResult::success(
      manipulation::workflow::rank_grasp_candidates(observation, std::move(candidates)));
  } catch (const std::invalid_argument & error) {
    return GraspProviderResult::failure(TaskErrorCode::kInternalError, error.what());
  }
}

void MtcGraspProvider::request_stop() noexcept
{
  stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
  planner_->request_stop();
}

FixedStateMtcSnapshotPlanner::FixedStateMtcSnapshotPlanner(
  moveit::core::RobotModelConstPtr robot_model,
  std::string output_link_frame)
: FixedStateMtcSnapshotPlanner(
    std::move(robot_model), std::move(output_link_frame),
    detail::make_default_mtc_task_operations())
{
}

FixedStateMtcSnapshotPlanner::FixedStateMtcSnapshotPlanner(
  moveit::core::RobotModelConstPtr robot_model,
  std::string output_link_frame,
  detail::FixedStateMtcTaskOperations task_operations)
: robot_model_(std::move(robot_model)), output_link_frame_(std::move(output_link_frame)),
  task_operations_(std::move(task_operations))
{
  if (!robot_model_) {
    throw std::invalid_argument("FixedState MTC planner requires a robot model");
  }
  if (!task_operations_.initialize || !task_operations_.plan ||
    !task_operations_.preempt || !task_operations_.extract_candidates)
  {
    throw std::invalid_argument("FixedState MTC planner requires complete task operations");
  }
  if (output_link_frame_.empty() || !robot_model_->hasLinkModel(output_link_frame_)) {
    throw std::invalid_argument("FixedState MTC planner output link is not in the robot model");
  }
}

GraspProviderResult FixedStateMtcSnapshotPlanner::plan(
  const manipulation::workflow::ObjectObservation & observation,
  const manipulation::workflow::ObjectProfile &,
  const manipulation::workflow::PlanningSceneSnapshot & scene_snapshot,
  const MtcGraspProfileConfig & config,
  manipulation::workflow::TransferExecutionContext & context)
{
  const auto request_generation = stop_generation_.load(std::memory_order_acquire);
  if (context.stop_requested()) {
    return GraspProviderResult::failure(TaskErrorCode::kCanceled, "grasp generation was canceled");
  }

  try {
    auto local_scene = detail::make_local_snapshot_scene(robot_model_, scene_snapshot);
    if (!local_scene->knowsFrameTransform(observation.pose().header.frame_id)) {
      return GraspProviderResult::failure(
        TaskErrorCode::kTfError,
        "MTC scene cannot resolve the workflow planning frame '" +
        observation.pose().header.frame_id + "'");
    }
    const auto grasp_reference_frame = detail::add_workflow_aligned_grasp_frame(
      local_scene, observation.object_id(), observation.pose().header.frame_id);
    auto task = detail::make_fixed_state_grasp_task(
      local_scene, grasp_reference_frame, config);
    try {
      task_operations_.initialize(task);
    } catch (const mtc::InitStageException & error) {
      // InitStageException retains pointers to stages owned by task, so render
      // the diagnostic before task leaves this scope.
      return GraspProviderResult::failure(
        TaskErrorCode::kPlanningFailed, init_stage_error_detail(error));
    }
    manipulation::task::TaskStateSnapshot waiting;
    waiting.phase = manipulation::task::TaskPhase::kPlanning;
    waiting.detail = observation.object_id() + ": generating ranked grasps on scene revision " +
      std::to_string(scene_snapshot.revision());
    context.report_waiting(
      manipulation::workflow::TransferWaitingReason::kGraspGeneration, waiting);
    const auto plan_result = detail::run_interruptible_mtc_plan(
      [this, &task, &config]() {
        return task_operations_.plan(task, config.max_solutions);
      },
      [this, &context, request_generation]() {
        return stop_generation_.load(std::memory_order_acquire) != request_generation ||
               context.checkpoint() ==
               manipulation::workflow::TransferExecutionControlResult::kStopped;
      },
      [this, &task]() {task_operations_.preempt(task);});
    waiting.detail = observation.object_id() + ": grasp generation finished";
    context.report_waiting(manipulation::workflow::TransferWaitingReason::kNone, waiting);
    if (plan_result.stopped ||
      stop_generation_.load(std::memory_order_acquire) != request_generation ||
      context.stop_requested())
    {
      return GraspProviderResult::failure(TaskErrorCode::kCanceled,
          "grasp generation was canceled");
    }
    if (!plan_result.planned) {
      return GraspProviderResult::failure(
        TaskErrorCode::kPlanningFailed,
          "MTC found no grasp candidate in the supplied scene revision");
    }

    auto candidates = task_operations_.extract_candidates(
      task, observation, config, output_link_frame_);
    if (candidates.empty()) {
      return GraspProviderResult::failure(
        TaskErrorCode::kPlanningFailed, "MTC solutions contained no usable grasp state");
    }
    return GraspProviderResult::success(std::move(candidates));
  } catch (const std::exception & error) {
    return GraspProviderResult::failure(TaskErrorCode::kInternalError, error.what());
  }
}

void FixedStateMtcSnapshotPlanner::request_stop() noexcept
{
  stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
}

namespace detail
{

geometry_msgs::msg::PoseStamped express_link_pose_in_frame(
  const Eigen::Isometry3d & model_to_output_frame,
  const Eigen::Isometry3d & model_to_link_frame,
  const Eigen::Isometry3d & link_to_pose,
  std_msgs::msg::Header output_header)
{
  geometry_msgs::msg::PoseStamped result;
  result.header = std::move(output_header);
  result.pose = franka_motion::detail::eigen_to_pose(
    model_to_output_frame.inverse() * model_to_link_frame * link_to_pose);
  return result;
}

FixedStateMtcTaskOperations make_default_mtc_task_operations()
{
  FixedStateMtcTaskOperations operations;
  operations.initialize = [](mtc::Task & task) {
      task.init();
      task.resetPreemptRequest();
    };
  operations.plan = [](mtc::Task & task, const std::size_t max_solutions) {
      return static_cast<bool>(task.plan(max_solutions));
    };
  operations.preempt = [](mtc::Task & task) {task.preempt();};
  operations.extract_candidates = [](
    mtc::Task & task,
    const manipulation::workflow::ObjectObservation & observation,
    const MtcGraspProfileConfig & config,
    const std::string & output_link_frame)
    {
      std::vector<GraspCandidate> candidates;
      candidates.reserve(task.solutions().size());
      for (const auto & solution : task.solutions()) {
        if (!std::isfinite(solution->cost())) {
          continue;
        }
        candidates.emplace_back(candidate_from_solution(
            *solution, observation, config, output_link_frame));
      }
      return candidates;
    };
  return operations;
}

InterruptibleMtcPlanResult run_interruptible_mtc_plan(
  std::function<bool()> plan,
  std::function<bool()> stop_requested,
  std::function<void()> preempt)
{
  if (!plan || !stop_requested || !preempt) {
    throw std::invalid_argument("interruptible MTC plan callbacks must not be empty");
  }

  std::atomic<bool> stopped{false};
  franka_motion::detail::MtcStopMonitor stop_monitor(
    [&stop_requested, &stopped]() {
      try {
        if (stop_requested()) {
          stopped.store(true, std::memory_order_release);
          return true;
        }
        return false;
      } catch (...) {
        stopped.store(true, std::memory_order_release);
        throw;
      }
    },
    std::move(preempt));
  const bool planned = plan();
  stop_monitor.finish();
  return InterruptibleMtcPlanResult{
    planned, stopped.load(std::memory_order_acquire)};
}

planning_scene::PlanningScenePtr make_local_snapshot_scene(
  const moveit::core::RobotModelConstPtr & robot_model,
  const manipulation::workflow::PlanningSceneSnapshot & snapshot)
{
  if (!robot_model) {
    throw std::invalid_argument("local PlanningScene requires a robot model");
  }
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  if (!scene->setPlanningSceneMsg(snapshot.scene())) {
    throw std::invalid_argument("complete PlanningScene snapshot is incompatible with robot model");
  }
  return scene;
}

std::string add_workflow_aligned_grasp_frame(
  const planning_scene::PlanningScenePtr & local_scene,
  const std::string & object_id,
  const std::string & workflow_frame)
{
  constexpr const char * kSubframeName = "__mtc_workflow_grasp_reference";
  if (!local_scene || object_id.empty() || workflow_frame.empty()) {
    throw std::invalid_argument("workflow-aligned grasp frame input is invalid");
  }
  const auto world = local_scene->getWorldNonConst();
  const auto object = world->getObject(object_id);
  if (!object) {
    throw std::invalid_argument(
            "MTC scene does not contain observed object '" + object_id + "'");
  }
  auto & robot_state = local_scene->getCurrentStateNonConst();
  if (!local_scene->knowsFrameTransform(robot_state, workflow_frame)) {
    throw std::invalid_argument(
            "MTC scene cannot resolve workflow grasp frame '" + workflow_frame + "'");
  }

  const Eigen::Isometry3d model_to_object = object->pose_;
  const Eigen::Isometry3d model_to_workflow = local_scene->getFrameTransform(
    robot_state, workflow_frame);
  Eigen::Isometry3d model_to_reference = Eigen::Isometry3d::Identity();
  model_to_reference.translation() = model_to_object.translation();
  model_to_reference.linear() = model_to_workflow.linear();

  auto subframes = object->subframe_poses_;
  if (!subframes.emplace(
      kSubframeName, model_to_object.inverse() * model_to_reference).second)
  {
    throw std::invalid_argument(
            "observed object uses the reserved MTC grasp subframe name");
  }
  if (!world->setSubframesOfObject(object_id, subframes)) {
    throw std::runtime_error("failed to add local workflow-aligned MTC grasp frame");
  }
  const std::string reference_frame = object_id + "/" + kSubframeName;
  if (!local_scene->knowsFrameTransform(robot_state, reference_frame)) {
    throw std::runtime_error("local workflow-aligned MTC grasp frame is unavailable");
  }
  return reference_frame;
}

mtc::Task make_fixed_state_grasp_task(
  const planning_scene::PlanningScenePtr & local_scene,
  const std::string & grasp_reference_frame,
  const MtcGraspProfileConfig & config)
{
  if (!local_scene || grasp_reference_frame.empty() || !is_valid_config(config) ||
    !local_scene->knowsFrameTransform(grasp_reference_frame))
  {
    throw std::invalid_argument("FixedState MTC task input is invalid");
  }
  mtc::Task task("snapshot grasp provider");
  task.stages()->setName("snapshot grasp provider");
  task.setRobotModel(local_scene->getRobotModel());
  task.setProperty("group", config.arm_group_name);
  task.setProperty("eef", config.eef_name);
  task.setProperty("ik_frame", config.ik_frame_link);

  auto fixed_state = std::make_unique<mtc::stages::FixedState>("snapshot", local_scene);
  mtc::Stage * monitored_state = fixed_state.get();
  task.add(std::move(fixed_state));
  task.add(std::make_unique<GraspCandidateConnector>());

  auto generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
  generator->properties().configureInitFrom(mtc::Stage::PARENT);
  generator->setEndEffector(config.eef_name);
  generator->setPreGraspPose(config.pregrasp_pose);
  generator->setGraspPose(config.grasp_pose);
  generator->setObject(grasp_reference_frame);
  generator->setAngleDelta(config.angle_delta);
  generator->setRotationAxis(config.rotation_axis);
  generator->setMonitoredStage(monitored_state);

  auto compute_ik = std::make_unique<mtc::stages::ComputeIK>(
    "grasp pose IK", std::move(generator));
  compute_ik->setEndEffector(config.eef_name);
  compute_ik->setGroup(config.arm_group_name);
  compute_ik->setIKFrame(config.ik_frame_transform, config.ik_frame_link);
  compute_ik->setMaxIKSolutions(static_cast<std::uint32_t>(config.max_ik_solutions));
  compute_ik->setMinSolutionDistance(config.min_solution_distance);
  compute_ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
  compute_ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

  task.add(std::move(compute_ik));
  return task;
}

}  // namespace detail
}  // namespace franka_motion::workflow_v2
