// Responsibility: Executes the direct pick/lift/place sequence with typed evidence and recovery.

#include "manipulation_moveit/pick_lift_place_strategy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "Eigen/Geometry"

#include "manipulation/task/task_status.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"
#include "manipulation/workflow/strategy_registry.hpp"

#include "trajectory_state_validation.hpp"

namespace manipulation::moveit
{
namespace
{

using Context = manipulation::workflow::TransferExecutionContext;
using ControlResult = manipulation::workflow::TransferExecutionControlResult;
using Disposition = manipulation::task::ObjectDisposition;
using ErrorCode = manipulation::task::TaskErrorCode;
using Phase = manipulation::task::TaskPhase;
using ReleaseDisposition = manipulation::task::ResourceReleaseDisposition;
using RecoveryPolicy = manipulation::workflow::TransferRecoveryPolicy;
using ResolvedTask = manipulation::workflow::ResolvedTransferTask;
using SceneLease = manipulation::workflow::PlanningSceneLease;
using WaitingReason = manipulation::workflow::TransferWaitingReason;

constexpr std::uint16_t kStepCount = 13U;
constexpr double kExecutedJointStateTolerance = 0.01;
constexpr double kPreDispatchJointStateTolerance = 0.01;
constexpr std::size_t kMaximumStartStatePlans = 3U;
constexpr auto kRobotStateSyncWaitSlice = std::chrono::milliseconds{5};

enum class GripperEffect
{
  kPrepareOpen,
  kGraspObject,
  kReleaseAtTarget,
};

struct StepFailure
{
  ErrorCode code{ErrorCode::kInternalError};
  std::string detail;
  ReleaseDisposition release{ReleaseDisposition::kReleaseSafe};
  bool stopped{false};
  bool scene_uncertain{false};
};

struct Waypoints
{
  geometry_msgs::msg::PoseStamped pregrasp;
  geometry_msgs::msg::PoseStamped grasp;
  geometry_msgs::msg::PoseStamped lift;
  geometry_msgs::msg::PoseStamped preplace;
  geometry_msgs::msg::PoseStamped place;
  geometry_msgs::msg::PoseStamped retreat;
};

struct TerminalJointState
{
  std::vector<std::string> names;
  std::vector<double> positions;
};

enum class RobotStateSyncStatus : std::uint8_t
{
  kReady,
  kPending,
  kInvalid,
};

bool finite_nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

bool finite_positive(const double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);
  return result;
}

geometry_msgs::msg::Pose from_isometry(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.linear());
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

geometry_msgs::msg::PoseStamped offset_z(
  geometry_msgs::msg::PoseStamped pose,
  const double distance)
{
  pose.pose.position.z += distance;
  return pose;
}

geometry_msgs::msg::PoseStamped offset_along(
  geometry_msgs::msg::PoseStamped pose,
  const Eigen::Vector3d & unit_direction,
  const double distance)
{
  pose.pose.position.x += unit_direction.x() * distance;
  pose.pose.position.y += unit_direction.y() * distance;
  pose.pose.position.z += unit_direction.z() * distance;
  return pose;
}

Waypoints make_waypoints(
  const ResolvedTask & task,
  const PickLiftPlaceStrategyConfig & config)
{
  const auto & observation = task.observation().pose();
  const auto & grasp = task.grasp();
  const auto & target = task.target_object_pose();
  const auto & frame = observation.header.frame_id;
  if (grasp.pregrasp_pose().header.frame_id != frame ||
    grasp.grasp_pose().header.frame_id != frame ||
    grasp.approach_direction().header.frame_id != frame ||
    target.header.frame_id != frame)
  {
    throw std::invalid_argument(
            "pick/lift/place requires observation, grasp, approach, and target in one frame");
  }

  Waypoints result;
  result.pregrasp = grasp.pregrasp_pose();
  result.grasp = grasp.grasp_pose();
  const Eigen::Vector3d candidate_approach(
    grasp.approach_direction().vector.x,
    grasp.approach_direction().vector.y,
    grasp.approach_direction().vector.z);
  const Eigen::Vector3d pregrasp_to_grasp =
    to_isometry(result.grasp.pose).translation() -
    to_isometry(result.pregrasp.pose).translation();
  if (pregrasp_to_grasp.norm() <= manipulation::validation::kRigidPoseTolerance ||
    pregrasp_to_grasp.dot(candidate_approach) <=
    manipulation::validation::kRigidPoseTolerance)
  {
    throw std::invalid_argument(
            "grasp candidate pregrasp-to-grasp motion must follow approach_direction");
  }
  // Clearance waypoints depart along the reverse of the candidate approach.
  // The place side reuses the same direction: no independent approach is
  // resolved for the target support today.
  const Eigen::Vector3d departure = -candidate_approach.normalized();
  result.lift = offset_along(
    result.grasp, departure, config.task.lift_clearance_m);

  const auto observation_to_eef =
    to_isometry(observation.pose).inverse() * to_isometry(result.grasp.pose);
  result.place = target;
  result.place.pose = from_isometry(to_isometry(target.pose) * observation_to_eef);
  result.preplace = offset_along(
    result.place, departure, config.task.preplace_clearance_m);
  result.retreat = offset_along(
    result.place, departure, config.task.retreat_clearance_m);

  const geometry_msgs::msg::PoseStamped poses[] = {
    result.pregrasp, result.grasp, result.lift,
    result.preplace, result.place, result.retreat};
  for (const auto & pose : poses) {
    if (!manipulation::validation::is_valid_pose_stamped(pose)) {
      throw std::invalid_argument("pick/lift/place generated an invalid rigid waypoint");
    }
  }
  return result;
}

ErrorCode physical_error(
  const PhysicalCommandOutcome outcome,
  const bool gripper) noexcept
{
  switch (outcome) {
    case PhysicalCommandOutcome::kSuccess:
      return ErrorCode::kInternalError;
    case PhysicalCommandOutcome::kRejected:
    case PhysicalCommandOutcome::kFailed:
      return gripper ? ErrorCode::kGripperFailed : ErrorCode::kExecutionFailed;
    case PhysicalCommandOutcome::kUnavailable:
      return ErrorCode::kControllerUnavailable;
    case PhysicalCommandOutcome::kTimedOut:
      return ErrorCode::kTimeout;
    case PhysicalCommandOutcome::kCanceled:
    case PhysicalCommandOutcome::kShutdown:
      return ErrorCode::kCanceled;
    case PhysicalCommandOutcome::kInternalError:
      return ErrorCode::kInternalError;
  }
  return ErrorCode::kInternalError;
}

StepFailure physical_failure(
  const PhysicalCommandResult & result,
  const bool gripper)
{
  return {
    physical_error(result.outcome(), gripper), result.detail(),
    result.safety() == PhysicalCommandSafety::kActiveStateUnknown ?
    ReleaseDisposition::kQuarantineRequired : ReleaseDisposition::kReleaseSafe,
    result.outcome() == PhysicalCommandOutcome::kCanceled ||
    result.outcome() == PhysicalCommandOutcome::kShutdown,
    false};
}

StepFailure planning_failure(const ArmPlanningResult & result)
{
  switch (result.outcome()) {
    case PlanningOutcome::kSuccess:
      return {ErrorCode::kInternalError, "successful plan entered failure mapping"};
    case PlanningOutcome::kInvalidRequest:
    case PlanningOutcome::kInvalidStartState:
    case PlanningOutcome::kInvalidGoalState:
      return {ErrorCode::kInvalidGoal, result.detail()};
    case PlanningOutcome::kSceneRefreshRequired:
      return {ErrorCode::kSceneConflict, result.detail()};
    case PlanningOutcome::kSceneUnavailable:
      return {ErrorCode::kSceneFailed, result.detail()};
    case PlanningOutcome::kPlannerUnavailable:
      return {ErrorCode::kResourceUnavailable, result.detail()};
    case PlanningOutcome::kBudgetExhausted:
    case PlanningOutcome::kPlanningFailed:
      return {ErrorCode::kPlanningFailed, result.detail()};
    case PlanningOutcome::kCanceled:
    case PlanningOutcome::kShutdown:
      return {ErrorCode::kCanceled, result.detail(), ReleaseDisposition::kReleaseSafe, true};
    case PlanningOutcome::kInternalError:
      return {ErrorCode::kInternalError, result.detail()};
  }
  return {ErrorCode::kInternalError, "unknown planning outcome"};
}

StepFailure scene_failure(const CoordinatedSceneResult & result)
{
  switch (result.outcome()) {
    case CoordinatedSceneOutcome::kSuccess:
      return {ErrorCode::kInternalError, "successful scene result entered failure mapping"};
    case CoordinatedSceneOutcome::kConflict:
      return {ErrorCode::kSceneConflict, result.detail()};
    case CoordinatedSceneOutcome::kUnavailable:
    case CoordinatedSceneOutcome::kFailed:
      return {ErrorCode::kSceneFailed, result.detail()};
    case CoordinatedSceneOutcome::kCanceled:
    case CoordinatedSceneOutcome::kShutdown:
      return {ErrorCode::kCanceled, result.detail(), ReleaseDisposition::kReleaseSafe, true};
    case CoordinatedSceneOutcome::kStateUncertain:
      return {
        ErrorCode::kSceneFailed, result.detail(), ReleaseDisposition::kReleaseSafe,
        false, true};
    case CoordinatedSceneOutcome::kInternalError:
      return {ErrorCode::kInternalError, result.detail()};
  }
  return {ErrorCode::kInternalError, "unknown coordinated scene outcome"};
}

StepFailure health_failure(const PickLiftPlaceHealthResult & result)
{
  switch (result.outcome()) {
    case PickLiftPlaceHealthOutcome::kReady:
      return {ErrorCode::kInternalError, "ready health result entered failure mapping"};
    case PickLiftPlaceHealthOutcome::kUnavailable:
      return {ErrorCode::kResourceUnavailable, result.detail()};
    case PickLiftPlaceHealthOutcome::kCanceled:
    case PickLiftPlaceHealthOutcome::kShutdown:
      return {ErrorCode::kCanceled, result.detail(), ReleaseDisposition::kReleaseSafe, true};
    case PickLiftPlaceHealthOutcome::kInternalError:
      return {ErrorCode::kInternalError, result.detail()};
  }
  return {ErrorCode::kInternalError, "unknown pick/lift/place health outcome"};
}

bool full_scene(const manipulation::workflow::PlanningSceneSnapshot & snapshot) noexcept
{
  return !snapshot.scene().is_diff && !snapshot.scene().robot_state.is_diff;
}

std::optional<TerminalJointState> terminal_joint_state(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & error)
{
  error.clear();
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty()) {
    return std::nullopt;
  }
  const auto & positions = joint_trajectory.points.back().positions;
  if (joint_trajectory.joint_names.empty() ||
    joint_trajectory.joint_names.size() != positions.size())
  {
    error = "planned trajectory has no complete terminal joint state";
    return std::nullopt;
  }
  std::unordered_set<std::string> unique_names;
  for (std::size_t index = 0U; index < joint_trajectory.joint_names.size(); ++index) {
    if (joint_trajectory.joint_names[index].empty() ||
      !unique_names.insert(joint_trajectory.joint_names[index]).second ||
      !std::isfinite(positions[index]))
    {
      error = "planned trajectory terminal joint state is malformed";
      return std::nullopt;
    }
  }
  return TerminalJointState{joint_trajectory.joint_names, positions};
}

RobotStateSyncStatus robot_state_sync_status(
  const TerminalJointState & expected,
  const moveit_msgs::msg::RobotState & observed)
{
  const auto & names = observed.joint_state.name;
  const auto & positions = observed.joint_state.position;
  if (names.empty() || names.size() != positions.size()) {
    return RobotStateSyncStatus::kInvalid;
  }
  std::unordered_map<std::string, double> observed_positions;
  observed_positions.reserve(names.size());
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (names[index].empty() || !std::isfinite(positions[index]) ||
      !observed_positions.emplace(names[index], positions[index]).second)
    {
      return RobotStateSyncStatus::kInvalid;
    }
  }
  for (std::size_t index = 0U; index < expected.names.size(); ++index) {
    const auto found = observed_positions.find(expected.names[index]);
    if (found == observed_positions.cend()) {
      return RobotStateSyncStatus::kInvalid;
    }
    if (std::abs(found->second - expected.positions[index]) >
      kExecutedJointStateTolerance)
    {
      return RobotStateSyncStatus::kPending;
    }
  }
  return RobotStateSyncStatus::kReady;
}

}  // namespace

void validatePickLiftPlaceStrategyConfig(const PickLiftPlaceStrategyConfig & config)
{
  const auto & robot = config.robot;
  const auto & task = config.task;
  if (!manipulation::validation::is_canonical_relative_frame_id(robot.attachment_link) ||
    robot.touch_links.empty())
  {
    throw std::invalid_argument(
            "pick/lift/place attachment link and touch links must be configured");
  }
  std::unordered_set<std::string> unique_touch_links;
  for (const auto & link : robot.touch_links) {
    if (!manipulation::validation::is_canonical_relative_frame_id(link) ||
      !unique_touch_links.insert(link).second)
    {
      throw std::invalid_argument(
              "pick/lift/place touch links must be canonical and unique");
    }
  }
  if (unique_touch_links.count(robot.attachment_link) == 0U) {
    throw std::invalid_argument(
            "pick/lift/place touch links must include the attachment link");
  }
  if (!finite_positive(task.lift_clearance_m) ||
    !finite_positive(task.preplace_clearance_m) ||
    !finite_positive(task.retreat_clearance_m) ||
    !finite_nonnegative(robot.open_width_m))
  {
    throw std::invalid_argument(
            "pick/lift/place clearances and open width must be finite and valid");
  }
  if (task.release_settle < std::chrono::steady_clock::duration::zero() ||
    config.arm_motion_timeout <= std::chrono::steady_clock::duration::zero() ||
    config.gripper_command_timeout <= std::chrono::steady_clock::duration::zero())
  {
    throw std::invalid_argument(
            "release settle must be non-negative and both physical timeouts positive");
  }
  if (robot.home_joint_names.size() != robot.home_joint_positions.size()) {
    throw std::invalid_argument("pick/lift/place home target arrays are inconsistent");
  }
  std::unordered_set<std::string> unique_home_joints;
  for (std::size_t index = 0U; index < robot.home_joint_names.size(); ++index) {
    if (robot.home_joint_names[index].empty() ||
      !std::isfinite(robot.home_joint_positions[index]) ||
      !unique_home_joints.insert(robot.home_joint_names[index]).second)
    {
      throw std::invalid_argument("pick/lift/place home target is malformed");
    }
  }
}

class PickLiftPlaceStrategy::Impl final
{
public:
  Impl(
    PickLiftPlaceStrategyConfig config,
    ArmPlanningPort & planning,
    ITrajectoryExecutionPort & trajectory_execution,
    IStrategyGripperCommandPort & gripper,
    IPickLiftPlaceScenePort & scene,
    IPickLiftPlaceHealthPort & health)
  : config_(std::move(config)), planning_(planning),
    trajectory_execution_(trajectory_execution), gripper_(gripper), scene_(scene),
    health_(health)
  {
    validatePickLiftPlaceStrategyConfig(config_);
  }

  manipulation::task::TaskOutcome execute(
    const ResolvedTask & task,
    Context & context)
  {
    const std::lock_guard<std::mutex> lock(execute_mutex_);
    planning_.begin_goal();
    trajectory_execution_.begin_goal();
    gripper_.begin_goal();
    scene_.begin_goal();
    health_.begin_goal();
    Runner runner(
      config_, planning_, trajectory_execution_, gripper_, scene_, health_, task, context);
    return runner.run();
  }

  void request_stop() noexcept
  {
    planning_.request_stop(PlanningStopCause::kCanceled);
    trajectory_execution_.request_stop();
    gripper_.request_stop();
    scene_.request_stop();
    health_.request_stop();
  }

private:
  class Runner final
  {
public:
    Runner(
      const PickLiftPlaceStrategyConfig & config,
      ArmPlanningPort & planning,
      ITrajectoryExecutionPort & trajectory_execution,
      IStrategyGripperCommandPort & gripper,
      IPickLiftPlaceScenePort & scene,
      IPickLiftPlaceHealthPort & health,
      const ResolvedTask & task,
      Context & context)
    : config_(config), planning_(planning), trajectory_execution_(trajectory_execution),
      gripper_(gripper), scene_(scene), health_(health), task_(task), context_(context),
      current_scene_(SceneLease::create(task.scene_snapshot(), task.scene_access_proof()))
    {
    }

    manipulation::task::TaskOutcome run()
    {
      try {
        waypoints_ = make_waypoints(task_, config_);
        if (!full_scene(current_scene_.snapshot())) {
          return finish({
              ErrorCode::kInvalidGoal,
              "resolved task must contain a complete PlanningScene snapshot"});
        }
        if (const auto failure = gripper_step(
            GripperCommandKind::kOpen, config_.robot.open_width_m, 0.0, false,
            GripperPostcondition::kOpenConfirmed, Phase::kWaitingForResources,
            0.05F, 0U, GripperEffect::kPrepareOpen, "initial_open"))
        {
          return finish(*failure);
        }
        if (const auto failure = pose_motion(
            waypoints_->pregrasp, Phase::kApproaching, 0.12F, 1U, "pregrasp"))
        {
          return finish(*failure);
        }
        if (const auto failure = pose_motion(
            waypoints_->grasp, Phase::kApproaching, 0.25F, 2U,
            "approach_to_grasp"))
        {
          return finish(*failure);
        }
        if (const auto failure = gripper_step(
            GripperCommandKind::kClose, task_.grasp().gripper_opening_m(),
            task_.grasp().gripper_force_n(), config_.robot.allow_grasp_stall,
            GripperPostcondition::kClosedConfirmed, Phase::kGrasping,
            0.34F, 3U, GripperEffect::kGraspObject, "close"))
        {
          return finish(*failure);
        }
        if (const auto failure = attach()) {
          return finish(*failure);
        }
        if (const auto failure = pose_motion(
            waypoints_->lift, Phase::kLifting, 0.50F, 5U, "lift"))
        {
          return finish(*failure);
        }
        if (const auto failure = pose_motion(
            waypoints_->preplace, Phase::kTransferring, 0.60F, 6U, "preplace"))
        {
          return finish(*failure);
        }
        if (const auto failure = pose_motion(
            waypoints_->place, Phase::kPlacing, 0.70F, 7U, "place"))
        {
          return finish(*failure);
        }
        if (const auto failure = gripper_step(
            GripperCommandKind::kOpen, config_.robot.open_width_m, 0.0, false,
            GripperPostcondition::kOpenConfirmed, Phase::kReleasing,
            0.78F, 8U, GripperEffect::kReleaseAtTarget, "release"))
        {
          return finish(*failure);
        }
        if (const auto failure = detach()) {
          return finish(*failure);
        }
        report(Phase::kRetreating, 0.86F, 10U, "release_settle");
        if (context_.wait_for(config_.task.release_settle) == ControlResult::kStopped) {
          return finish(stopped("transfer stopped during release settle"));
        }
        if (const auto failure = pose_motion(
            waypoints_->retreat, Phase::kRetreating, 0.92F, 11U, "retreat"))
        {
          return finish(*failure);
        }
        if (task_.return_home()) {
          if (config_.robot.home_joint_names.empty()) {
            return finish({
                ErrorCode::kInvalidGoal,
                "return_home requires a configured joint target"});
          }
          if (const auto failure = joint_motion(
              config_.robot.home_joint_names, config_.robot.home_joint_positions,
              Phase::kRetreating, 0.97F, 12U, "home"))
          {
            return finish(*failure);
          }
        }
        if (const auto failure = checkpoint("transfer stopped before completion")) {
          return finish(*failure);
        }
        report(Phase::kCompleted, 1.0F, 12U, "pick/lift/place completed");
        return manipulation::task::TaskOutcome::make_success(
          Disposition::kObjectAtTarget, "pick/lift/place completed");
      } catch (const std::invalid_argument & error) {
        return finish({
            ErrorCode::kInvalidGoal,
            std::string{"pick/lift/place input is invalid: "} + error.what()});
      } catch (const std::exception & error) {
        return finish({
            ErrorCode::kInternalError,
            std::string{"pick/lift/place strategy threw: "} + error.what()});
      } catch (...) {
        return finish({
            ErrorCode::kInternalError,
            "pick/lift/place strategy threw an unknown exception"});
      }
    }

private:
    std::optional<StepFailure> checkpoint(const std::string & detail)
    {
      if (context_.checkpoint() == ControlResult::kStopped) {
        return stopped(detail);
      }
      return std::nullopt;
    }

    StepFailure stopped(std::string detail) const
    {
      return {
        ErrorCode::kCanceled, std::move(detail), ReleaseDisposition::kReleaseSafe,
        true, false};
    }

    void report(
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      std::string detail,
      const std::uint16_t retry_count = 0U)
    {
      context_.report_feedback(make_state(
          phase, progress, step, std::move(detail), retry_count));
    }

    manipulation::task::TaskStateSnapshot make_state(
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      std::string detail,
      const std::uint16_t retry_count = 0U) const
    {
      manipulation::task::TaskStateSnapshot state;
      state.phase = phase;
      state.progress = progress;
      state.step_index = step;
      state.step_count = kStepCount;
      state.retry_count = retry_count;
      state.detail = std::move(detail);
      return state;
    }

    void report_waiting(
      const WaitingReason reason,
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      std::string detail)
    {
      context_.report_waiting(
        reason, make_state(phase, progress, step, std::move(detail)));
    }

    void report_ready(
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      std::string detail,
      const std::uint16_t retry_count = 0U)
    {
      context_.report_waiting(
        WaitingReason::kNone,
        make_state(phase, progress, step, std::move(detail), retry_count));
    }

    std::optional<StepFailure> prepare_side_effect(
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      const std::string & label,
      const bool synchronize_executed_state = false)
    {
      while (true) {
        if (const auto failure = checkpoint("transfer stopped before " + label)) {
          return failure;
        }

        report_waiting(
          WaitingReason::kHealthReadiness, Phase::kWaitingForResources,
          progress, step, "waiting for production readiness before " + label);
        const auto health = health_.wait_until_ready(context_);
        if (!health.is_ready()) {
          if (context_.stop_requested()) {
            return stopped("transfer stopped while waiting for readiness before " + label);
          }
          return health_failure(health);
        }
        if (context_.stop_requested()) {
          return stopped("transfer stopped after readiness before " + label);
        }

        report_waiting(
          WaitingReason::kSceneReadiness, Phase::kWaitingForResources,
          progress, step, "revalidating scene before " + label);
        const auto result = scene_.revalidate(
          StrategySceneRevalidationRequest{
            current_scene_.snapshot(), current_scene_.access_proof()}, context_);
        if (!result.succeeded()) {
          if (context_.stop_requested()) {
            return stopped("transfer stopped during scene revalidation");
          }
          return scene_failure(result);
        }
        const auto & lease = *result.lease();
        const auto & expected_snapshot = current_scene_.snapshot();
        const auto & expected_proof = current_scene_.access_proof();
        if (!full_scene(lease.snapshot()) ||
          lease.access_proof().lease_id() != expected_proof.lease_id() ||
          lease.snapshot().coordinator_session_id() !=
          expected_snapshot.coordinator_session_id() ||
          lease.access_proof().coordinator_session_id() !=
          expected_proof.coordinator_session_id() ||
          lease.snapshot().revision() != expected_snapshot.revision() ||
          lease.access_proof().revision() != expected_proof.revision())
        {
          return StepFailure{
            ErrorCode::kSceneConflict,
            "scene lease/session/revision changed before a side effect"};
        }
        current_scene_ = lease;
        if (context_.stop_requested()) {
          return stopped("transfer stopped after scene revalidation before " + label);
        }

        if (synchronize_executed_state && expected_terminal_joint_state_) {
          const auto sync = robot_state_sync_status(
            *expected_terminal_joint_state_, current_scene_.snapshot().scene().robot_state);
          if (sync == RobotStateSyncStatus::kInvalid) {
            return StepFailure{
              ErrorCode::kInternalError,
              "complete PlanningScene cannot correlate the last executed joint state"};
          }
          if (sync == RobotStateSyncStatus::kPending) {
            report_waiting(
              WaitingReason::kPlanning, Phase::kWaitingForResources,
              progress, step,
              "waiting for PlanningScene robot state to reach the last trajectory endpoint");
            if (context_.wait_for(kRobotStateSyncWaitSlice) == ControlResult::kStopped) {
              return stopped("transfer stopped while synchronizing executed robot state");
            }
            continue;
          }
        }
        report_ready(phase, progress, step, "ready for " + label);
        return std::nullopt;
      }
    }

    ArmPlanningRequest pose_request(
      const geometry_msgs::msg::PoseStamped & target,
      std::string label) const
    {
      ArmPlanningRequest request;
      request.goal_kind = ArmPlanningGoalKind::kPose;
      request.target_pose = target;
      request.label = std::move(label);
      request.scene_revision = {
        current_scene_.snapshot().coordinator_session_id(),
        current_scene_.snapshot().revision()};
      request.scene_snapshot = current_scene_.snapshot().scene();
      return request;
    }

    ArmPlanningRequest joint_request(
      const std::vector<std::string> & names,
      const std::vector<double> & positions,
      std::string label) const
    {
      ArmPlanningRequest request;
      request.goal_kind = ArmPlanningGoalKind::kJoint;
      request.joint_names = names;
      request.joint_positions = positions;
      request.label = std::move(label);
      request.scene_revision = {
        current_scene_.snapshot().coordinator_session_id(),
        current_scene_.snapshot().revision()};
      request.scene_snapshot = current_scene_.snapshot().scene();
      return request;
    }

    void refresh_request_scene(ArmPlanningRequest & request) const
    {
      request.scene_revision = {
        current_scene_.snapshot().coordinator_session_id(),
        current_scene_.snapshot().revision()};
      request.scene_snapshot = current_scene_.snapshot().scene();
    }

    std::optional<StepFailure> plan_and_execute(
      ArmPlanningRequest request,
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      const std::string & label)
    {
      std::size_t completed_planning_attempts = 0U;
      for (std::size_t start_state_plan = 1U;
        start_state_plan <= kMaximumStartStatePlans; ++start_state_plan)
      {
        if (const auto failure = checkpoint("transfer stopped before planning " + label)) {
          return failure;
        }
        report_waiting(
          WaitingReason::kPlanning, Phase::kPlanning,
          progress, step, "planning " + label);
        const auto planned = planning_.plan(request, context_);
        if (!planned.succeeded()) {
          if (context_.stop_requested()) {
            return stopped("transfer stopped during planning " + label);
          }
          return planning_failure(planned);
        }
        completed_planning_attempts += std::max<std::size_t>(planned.attempt_count(), 1U);
        const auto total_retries = completed_planning_attempts - 1U;
        report_ready(
          phase, progress, step, "planned " + label,
          static_cast<std::uint16_t>(std::min<std::size_t>(
              total_retries, std::numeric_limits<std::uint16_t>::max())));
        if (planned.planned_scene_revision() != request.scene_revision) {
          return StepFailure{
            ErrorCode::kSceneConflict,
            "planner result does not identify the requested scene revision"};
        }
        if (planned.trajectory().joint_trajectory.points.empty() &&
          planned.trajectory().multi_dof_joint_trajectory.points.empty())
        {
          return StepFailure{
            ErrorCode::kInternalError,
            "planner returned success with an empty trajectory"};
        }
        std::string terminal_state_error;
        auto terminal_state = terminal_joint_state(
          planned.trajectory(), terminal_state_error);
        if (!terminal_state_error.empty()) {
          return StepFailure{ErrorCode::kInternalError, std::move(terminal_state_error)};
        }
        if (const auto failure = prepare_side_effect(
            phase, progress, step, "trajectory " + label))
        {
          return failure;
        }

        const auto start_check = detail::check_trajectory_start_state(
          current_scene_.snapshot().scene().robot_state,
          planned.trajectory(), kPreDispatchJointStateTolerance);
        if (start_check.status == detail::TrajectoryStartStateStatus::kInvalid) {
          return StepFailure{
            ErrorCode::kInternalError,
            "cannot validate planned trajectory immediately before dispatch: " +
            start_check.detail};
        }
        if (start_check.status == detail::TrajectoryStartStateStatus::kMismatch) {
          if (start_state_plan == kMaximumStartStatePlans) {
            return StepFailure{
              ErrorCode::kPlanningFailed,
              "robot state kept changing between planning and trajectory dispatch"};
          }
          report_waiting(
            WaitingReason::kPlanning, Phase::kPlanning, progress, step,
            "robot state changed after planning; discarding trajectory and replanning " +
            label);
          if (context_.wait_for(kRobotStateSyncWaitSlice) == ControlResult::kStopped) {
            return stopped("transfer stopped while replanning a changed trajectory start");
          }
          refresh_request_scene(request);
          continue;
        }

        report(
          phase, progress, step, "executing " + label,
          static_cast<std::uint16_t>(std::min<std::size_t>(
              total_retries, std::numeric_limits<std::uint16_t>::max())));
        expected_terminal_joint_state_.reset();
        const auto executed = trajectory_execution_.execute(
          TrajectoryExecutionRequest{planned.trajectory(), label},
          config_.arm_motion_timeout, context_);
        if (!executed.succeeded()) {
          if (context_.stop_requested() &&
            executed.safety() != PhysicalCommandSafety::kActiveStateUnknown)
          {
            return stopped("transfer stopped during trajectory " + label);
          }
          return physical_failure(executed, false);
        }
        if (executed.safety() != PhysicalCommandSafety::kTerminalSafe) {
          return StepFailure{
            ErrorCode::kInternalError,
            "successful trajectory execution did not prove a terminal-safe state",
            ReleaseDisposition::kQuarantineRequired};
        }
        expected_terminal_joint_state_ = std::move(terminal_state);
        return checkpoint("transfer stopped after trajectory " + label);
      }
      return StepFailure{
        ErrorCode::kInternalError,
        "trajectory start-state replan loop exhausted without a terminal result"};
    }

    std::optional<StepFailure> pose_motion(
      const geometry_msgs::msg::PoseStamped & target,
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      const std::string & label)
    {
      if (const auto failure = prepare_side_effect(
          phase, progress, step, "planning input " + label, true))
      {
        return failure;
      }
      const auto failure = plan_and_execute(
        pose_request(target, label), phase, progress, step, label);
      if (!failure) {
        last_pose_ = target;
      }
      return failure;
    }

    std::optional<StepFailure> joint_motion(
      const std::vector<std::string> & names,
      const std::vector<double> & positions,
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      const std::string & label)
    {
      if (const auto failure = prepare_side_effect(
          phase, progress, step, "planning input " + label, true))
      {
        return failure;
      }
      return plan_and_execute(
        joint_request(names, positions, label), phase, progress, step, label);
    }

    std::optional<StepFailure> gripper_step(
      const GripperCommandKind kind,
      const double width,
      const double effort,
      const bool allow_stall,
      const GripperPostcondition expected,
      const Phase phase,
      const float progress,
      const std::uint16_t step,
      const GripperEffect effect,
      const std::string & label)
    {
      if (const auto failure = prepare_side_effect(
          phase, progress, step, "gripper " + label))
      {
        return failure;
      }
      report(phase, progress, step, "gripper " + label);
      const auto result = gripper_.command(
        StrategyGripperCommandRequest{
          kind, width, effort, allow_stall, label},
        config_.gripper_command_timeout, context_);
      if (!result.command().succeeded()) {
        if (result.command().safety() != PhysicalCommandSafety::kNotAccepted &&
          result.postcondition() == GripperPostcondition::kUnknown &&
          effect != GripperEffect::kPrepareOpen)
        {
          disposition_ = Disposition::kUnknown;
        }
        if (context_.stop_requested() &&
          result.command().safety() != PhysicalCommandSafety::kActiveStateUnknown)
        {
          return stopped("transfer stopped during gripper " + label);
        }
        return physical_failure(result.command(), true);
      }
      if (result.postcondition() != expected ||
        result.command().safety() != PhysicalCommandSafety::kTerminalSafe)
      {
        return StepFailure{
          ErrorCode::kInternalError,
          "successful gripper command returned inconsistent postcondition evidence",
          result.command().safety() == PhysicalCommandSafety::kActiveStateUnknown ?
          ReleaseDisposition::kQuarantineRequired : ReleaseDisposition::kReleaseSafe};
      }
      // Commit terminal physical evidence before the next checkpoint can stop the task.
      if (effect == GripperEffect::kGraspObject) {
        disposition_ = Disposition::kHoldingObject;
      } else if (effect == GripperEffect::kReleaseAtTarget) {
        disposition_ = Disposition::kObjectAtTarget;
        release_pending_detach_ = true;
      }
      return checkpoint("transfer stopped after gripper " + label);
    }

    std::optional<StepFailure> attach()
    {
      if (disposition_ != Disposition::kHoldingObject) {
        return StepFailure{
          ErrorCode::kInternalError,
          "scene attach requires a confirmed closed-gripper holding state"};
      }
      if (const auto failure = prepare_side_effect(
          Phase::kAttaching, 0.43F, 4U, "scene attach"))
      {
        return failure;
      }
      report(Phase::kAttaching, 0.43F, 4U, "scene attach");
      const auto previous_revision = current_scene_.snapshot().revision();
      const auto result = scene_.attach(
        StrategySceneAttachRequest{
          current_scene_.snapshot(), current_scene_.access_proof(),
          task_.instance().object_id(), config_.robot.attachment_link,
          config_.robot.touch_links}, context_);
      if (!result.succeeded()) {
        auto failure = scene_failure(result);
        if (failure.scene_uncertain) {
          disposition_ = Disposition::kSceneUncertain;
          return failure;
        }
        if (context_.stop_requested()) {
          return stopped("transfer stopped during scene attach");
        }
        return failure;
      }
      if (const auto failure = accept_mutation_lease(*result.lease(), previous_revision)) {
        disposition_ = Disposition::kSceneUncertain;
        return failure;
      }
      return checkpoint("transfer stopped after scene attach");
    }

    std::optional<StepFailure> detach()
    {
      if (disposition_ != Disposition::kObjectAtTarget) {
        return StepFailure{
          ErrorCode::kInternalError,
          "scene detach requires a confirmed physical release at target"};
      }
      if (const auto failure = prepare_side_effect(
          Phase::kDetaching, 0.83F, 9U, "scene detach"))
      {
        return failure;
      }
      report(Phase::kDetaching, 0.83F, 9U, "scene detach");
      const auto previous_revision = current_scene_.snapshot().revision();
      const auto result = scene_.detach(
        StrategySceneDetachRequest{
          current_scene_.snapshot(), current_scene_.access_proof(),
          task_.instance().object_id(), config_.robot.attachment_link,
          task_.target_object_pose()}, context_);
      if (!result.succeeded()) {
        auto failure = scene_failure(result);
        if (failure.scene_uncertain) {
          disposition_ = Disposition::kSceneUncertain;
          return failure;
        }
        if (context_.stop_requested()) {
          return stopped("transfer stopped during scene detach");
        }
        return failure;
      }
      if (const auto failure = accept_mutation_lease(*result.lease(), previous_revision)) {
        disposition_ = Disposition::kSceneUncertain;
        return failure;
      }
      release_pending_detach_ = false;
      return checkpoint("transfer stopped after scene detach");
    }

    std::optional<StepFailure> accept_mutation_lease(
      const SceneLease & lease,
      const std::uint64_t previous_revision)
    {
      if (!full_scene(lease.snapshot()) ||
        lease.access_proof().lease_id() != current_scene_.access_proof().lease_id() ||
        lease.snapshot().coordinator_session_id() !=
        current_scene_.snapshot().coordinator_session_id() ||
        lease.access_proof().coordinator_session_id() !=
        current_scene_.access_proof().coordinator_session_id() ||
        lease.snapshot().revision() <= previous_revision ||
        lease.access_proof().revision() != lease.snapshot().revision())
      {
        return StepFailure{
          ErrorCode::kSceneFailed,
          "scene mutation success returned invalid next full-scene lease",
          ReleaseDisposition::kReleaseSafe, false, true};
      }
      current_scene_ = lease;
      return std::nullopt;
    }

    manipulation::task::TaskOutcome finish(StepFailure failure)
    {
      if (release_pending_detach_) {
        // The gripper is terminal-safe, but releasing ownership would expose an attached
        // scene object that has physically been released. Keep the scene lease for recovery.
        failure.scene_uncertain = true;
        failure.detail += "; gripper release at target confirmed, scene detach unconfirmed";
      }
      if (failure.scene_uncertain) {
        disposition_ = Disposition::kSceneUncertain;
      }
      if (failure.stopped || context_.stop_requested()) {
        request_all_stop();
        return manipulation::task::TaskOutcome::make_failure(
          ErrorCode::kCanceled, disposition_, 0U,
          failure.detail.empty() ? "pick/lift/place was canceled" : failure.detail,
          failure.release);
      }
      if (failure.release == ReleaseDisposition::kQuarantineRequired) {
        request_all_stop();
        return manipulation::task::TaskOutcome::make_failure(
          failure.code, disposition_, 0U, failure.detail, failure.release);
      }

      const auto policy = task_.recovery_policy();
      if (policy == RecoveryPolicy::kNone) {
        return manipulation::task::TaskOutcome::make_failure(
          failure.code, disposition_, 0U, failure.detail, failure.release);
      }
      report(Phase::kRecovering, 0.98F, 13U, "starting bounded recovery");
      if (policy == RecoveryPolicy::kSafeStop) {
        request_all_stop();
        return manipulation::task::TaskOutcome::make_failure(
          failure.code, disposition_, 1U,
          failure.detail + "; safe-stop requested", failure.release);
      }
      if (failure.code == ErrorCode::kSceneConflict ||
        disposition_ == Disposition::kSceneUncertain ||
        disposition_ == Disposition::kUnknown || !last_pose_)
      {
        request_all_stop();
        return manipulation::task::TaskOutcome::make_failure(
          ErrorCode::kRecoveryFailed, disposition_, 1U,
          failure.detail + "; motion recovery was not safe from current evidence",
          failure.release);
      }

      auto recovery_pose = offset_z(*last_pose_, config_.task.retreat_clearance_m);
      if (const auto recovery_failure = pose_motion(
          recovery_pose, Phase::kRecovering, 0.985F, 13U, "recovery_retreat"))
      {
        request_all_stop();
        if (recovery_failure->stopped || context_.stop_requested()) {
          return manipulation::task::TaskOutcome::make_failure(
            ErrorCode::kCanceled, disposition_, 1U,
            failure.detail + "; recovery retreat was canceled",
            recovery_failure->release);
        }
        return manipulation::task::TaskOutcome::make_failure(
          ErrorCode::kRecoveryFailed, disposition_, 1U,
          failure.detail + "; recovery retreat failed: " + recovery_failure->detail,
          recovery_failure->release);
      }
      if (policy == RecoveryPolicy::kRetreatAndHome) {
        if (config_.robot.home_joint_names.empty()) {
          request_all_stop();
          return manipulation::task::TaskOutcome::make_failure(
            ErrorCode::kRecoveryFailed, disposition_, 1U,
            failure.detail + "; recovery home target is not configured");
        }
        if (const auto recovery_failure = joint_motion(
            config_.robot.home_joint_names, config_.robot.home_joint_positions,
            Phase::kRecovering, 0.99F, 13U, "recovery_home"))
        {
          request_all_stop();
          if (recovery_failure->stopped || context_.stop_requested()) {
            return manipulation::task::TaskOutcome::make_failure(
              ErrorCode::kCanceled, disposition_, 1U,
              failure.detail + "; recovery home was canceled",
              recovery_failure->release);
          }
          return manipulation::task::TaskOutcome::make_failure(
            ErrorCode::kRecoveryFailed, disposition_, 1U,
            failure.detail + "; recovery home failed: " + recovery_failure->detail,
            recovery_failure->release);
        }
      }
      return manipulation::task::TaskOutcome::make_failure(
        failure.code, disposition_, 1U,
        failure.detail + "; requested motion recovery completed", failure.release);
    }

    void request_all_stop() noexcept
    {
      planning_.request_stop(PlanningStopCause::kCanceled);
      trajectory_execution_.request_stop();
      gripper_.request_stop();
      scene_.request_stop();
      health_.request_stop();
    }

    const PickLiftPlaceStrategyConfig & config_;
    ArmPlanningPort & planning_;
    ITrajectoryExecutionPort & trajectory_execution_;
    IStrategyGripperCommandPort & gripper_;
    IPickLiftPlaceScenePort & scene_;
    IPickLiftPlaceHealthPort & health_;
    const ResolvedTask & task_;
    Context & context_;
    SceneLease current_scene_;
    std::optional<Waypoints> waypoints_;
    std::optional<geometry_msgs::msg::PoseStamped> last_pose_;
    std::optional<TerminalJointState> expected_terminal_joint_state_;
    Disposition disposition_{Disposition::kObjectAtSource};
    bool release_pending_detach_{false};
  };

  const PickLiftPlaceStrategyConfig config_;
  ArmPlanningPort & planning_;
  ITrajectoryExecutionPort & trajectory_execution_;
  IStrategyGripperCommandPort & gripper_;
  IPickLiftPlaceScenePort & scene_;
  IPickLiftPlaceHealthPort & health_;
  std::mutex execute_mutex_;
};

PickLiftPlaceStrategy::PickLiftPlaceStrategy(
  PickLiftPlaceStrategyConfig config,
  ArmPlanningPort & planning,
  ITrajectoryExecutionPort & trajectory_execution,
  IStrategyGripperCommandPort & gripper,
  IPickLiftPlaceScenePort & scene,
  IPickLiftPlaceHealthPort & health)
: impl_(std::make_unique<Impl>(
    std::move(config), planning, trajectory_execution, gripper, scene, health))
{
}

PickLiftPlaceStrategy::~PickLiftPlaceStrategy() = default;

std::string_view PickLiftPlaceStrategy::id() const noexcept
{
  return manipulation::workflow::kPickLiftPlaceStrategyId;
}

manipulation::task::TaskOutcome PickLiftPlaceStrategy::execute(
  const ResolvedTask & task,
  Context & context)
{
  try {
    return impl_->execute(task, context);
  } catch (const std::exception & error) {
    return manipulation::task::TaskOutcome::make_failure(
      ErrorCode::kInternalError, Disposition::kUnknown, 0U,
      std::string{"pick/lift/place setup failed closed: "} + error.what());
  } catch (...) {
    return manipulation::task::TaskOutcome::make_failure(
      ErrorCode::kInternalError, Disposition::kUnknown, 0U,
      "pick/lift/place setup failed with an unknown exception");
  }
}

void PickLiftPlaceStrategy::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::moveit
