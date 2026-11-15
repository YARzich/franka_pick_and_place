// Responsibility: Builds validated plan-only MoveGroup goals and maps terminal results.

#include "manipulation_moveit/moveit_arm_planning_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "moveit/kinematic_constraints/utils.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_call.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::moveit
{
namespace
{

using MoveGroup = moveit_msgs::action::MoveGroup;
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
using ResultCode = rclcpp_action::ResultCode;

// Deliberately implementation-owned: it must not become a task/backend profile timeout.
constexpr double kInternalPlanningBudgetSeconds = 5.0;

bool finite_scaling(const double value) noexcept
{
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

bool finite_positive(const double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

ArmPlanningResult stopped_result(
  const PlanningStopToken & stop_token,
  std::string fallback_detail)
{
  switch (stop_token.cause()) {
    case PlanningStopCause::kNone:
    case PlanningStopCause::kCanceled:
      return ArmPlanningResult::failure(
        PlanningOutcome::kCanceled, std::move(fallback_detail));
    case PlanningStopCause::kShutdown:
      return ArmPlanningResult::failure(
        PlanningOutcome::kShutdown, "ROS planning owner is shutting down");
  }
  return ArmPlanningResult::failure(
    PlanningOutcome::kInternalError, "unknown planning stop cause");
}

ArmPlanningResult transport_failure(
  const detail::ActionCallStatus status,
  const PlanningStopToken & stop_token,
  std::string detail_text)
{
  switch (status) {
    case detail::ActionCallStatus::kResultReceived:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError,
        "MoveGroup transport mapper received a result without a payload");
    case detail::ActionCallStatus::kCanceled:
      return stopped_result(stop_token, std::move(detail_text));
    case detail::ActionCallStatus::kShutdown:
      return ArmPlanningResult::failure(
        PlanningOutcome::kShutdown, std::move(detail_text));
    case detail::ActionCallStatus::kUnavailable:
    case detail::ActionCallStatus::kRejected:
      return ArmPlanningResult::failure(
        PlanningOutcome::kPlannerUnavailable, std::move(detail_text));
    case detail::ActionCallStatus::kTimeout:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError,
        "MoveGroup transport wait unexpectedly exhausted its unbounded deadline");
    case detail::ActionCallStatus::kInternalError:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, std::move(detail_text));
  }
  return ArmPlanningResult::failure(
    PlanningOutcome::kInternalError, "unknown MoveGroup transport result");
}

ArmPlanningResult moveit_failure(
  const std::int32_t code,
  const PlanningStopToken & stop_token,
  std::string detail_text)
{
  switch (code) {
    case MoveItErrorCodes::TIMED_OUT:
      return ArmPlanningResult::failure(
        PlanningOutcome::kBudgetExhausted, std::move(detail_text), code);
    case MoveItErrorCodes::PREEMPTED:
      return stopped_result(stop_token, std::move(detail_text));
    case MoveItErrorCodes::START_STATE_IN_COLLISION:
    case MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
    case MoveItErrorCodes::START_STATE_INVALID:
    case MoveItErrorCodes::INVALID_ROBOT_STATE:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInvalidStartState, std::move(detail_text), code);
    case MoveItErrorCodes::GOAL_IN_COLLISION:
    case MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
    case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
    case MoveItErrorCodes::GOAL_STATE_INVALID:
    case MoveItErrorCodes::NO_IK_SOLUTION:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInvalidGoalState, std::move(detail_text), code);
    case MoveItErrorCodes::INVALID_GROUP_NAME:
    case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
    case MoveItErrorCodes::INVALID_LINK_NAME:
    case MoveItErrorCodes::INVALID_OBJECT_NAME:
    case MoveItErrorCodes::UNRECOGNIZED_GOAL_TYPE:
    case MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInvalidRequest, std::move(detail_text), code);
    case MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return ArmPlanningResult::failure(
        PlanningOutcome::kSceneRefreshRequired, std::move(detail_text), code);
    case MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
    case MoveItErrorCodes::ROBOT_STATE_STALE:
    case MoveItErrorCodes::SENSOR_INFO_STALE:
      return ArmPlanningResult::failure(
        PlanningOutcome::kSceneUnavailable, std::move(detail_text), code);
    case MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
    case MoveItErrorCodes::COMMUNICATION_FAILURE:
      return ArmPlanningResult::failure(
        PlanningOutcome::kPlannerUnavailable, std::move(detail_text), code);
    case MoveItErrorCodes::PLANNING_FAILED:
    case MoveItErrorCodes::INVALID_MOTION_PLAN:
    case MoveItErrorCodes::FAILURE:
    case MoveItErrorCodes::ABORT:
      return ArmPlanningResult::failure(
        PlanningOutcome::kPlanningFailed, std::move(detail_text), code);
    case MoveItErrorCodes::CONTROL_FAILED:
    case MoveItErrorCodes::CRASH:
    case MoveItErrorCodes::UNDEFINED:
    case MoveItErrorCodes::SUCCESS:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, std::move(detail_text), code);
    default:
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, std::move(detail_text), code);
  }
}

std::optional<MoveGroup::Goal> make_goal(
  const MoveItArmPlanningAdapterConfig & config,
  const ArmPlanningRequest & request,
  std::string & error)
{
  MoveGroup::Goal goal;
  goal.request.group_name = config.planning_group;
  goal.request.pipeline_id = config.planning_pipeline;
  goal.request.planner_id = config.planner_id;
  goal.request.num_planning_attempts = 1;
  goal.request.allowed_planning_time = kInternalPlanningBudgetSeconds;
  goal.request.max_velocity_scaling_factor = config.velocity_scaling;
  goal.request.max_acceleration_scaling_factor = config.acceleration_scaling;
  goal.request.start_state.is_diff = true;
  goal.planning_options.plan_only = true;
  if (request.scene_snapshot.is_diff || request.scene_snapshot.robot_state.is_diff) {
    error = "MoveGroup planning requires a complete immutable PlanningScene snapshot";
    return std::nullopt;
  }
  goal.planning_options.planning_scene_diff = request.scene_snapshot;

  switch (request.goal_kind) {
    case ArmPlanningGoalKind::kPose: {
        if (!manipulation::validation::is_valid_pose_stamped(request.target_pose)) {
          error = "pose planning target must be finite, normalized, stamped, and framed";
          return std::nullopt;
        }
        auto constraints = kinematic_constraints::constructGoalConstraints(
          config.end_effector_link, request.target_pose,
          config.position_tolerance, config.orientation_tolerance);
        constraints.name = request.label;
        goal.request.goal_constraints.push_back(std::move(constraints));
        break;
      }
    case ArmPlanningGoalKind::kJoint: {
        if (request.joint_names.empty() ||
          request.joint_names.size() != request.joint_positions.size())
        {
          error = "joint planning target names and positions must be non-empty and equally sized";
          return std::nullopt;
        }
        std::unordered_set<std::string> unique_names;
        moveit_msgs::msg::Constraints constraints;
        constraints.name = request.label;
        constraints.joint_constraints.reserve(request.joint_names.size());
        for (std::size_t index = 0U; index < request.joint_names.size(); ++index) {
          if (request.joint_names[index].empty() ||
            !std::isfinite(request.joint_positions[index]) ||
            !unique_names.insert(request.joint_names[index]).second)
          {
            error = "joint planning target names must be unique and positions finite";
            return std::nullopt;
          }
          moveit_msgs::msg::JointConstraint joint;
          joint.joint_name = request.joint_names[index];
          joint.position = request.joint_positions[index];
          joint.tolerance_above = config.joint_tolerance;
          joint.tolerance_below = config.joint_tolerance;
          joint.weight = 1.0;
          constraints.joint_constraints.push_back(std::move(joint));
        }
        goal.request.goal_constraints.push_back(std::move(constraints));
        break;
      }
  }
  if (goal.request.goal_constraints.empty()) {
    error = "arm planning goal kind did not produce any MoveIt constraints";
    return std::nullopt;
  }
  return goal;
}

std::string result_detail(const MoveGroup::Result & result)
{
  std::string detail_text = "MoveGroup planning failed";
  if (!result.error_code.message.empty()) {
    detail_text += ": " + result.error_code.message;
  }
  if (!result.error_code.source.empty()) {
    detail_text += " [source=" + result.error_code.source + "]";
  }
  return detail_text;
}

}  // namespace

void validateMoveItArmPlanningAdapterConfig(const MoveItArmPlanningAdapterConfig & config)
{
  if (config.move_group_action.empty() || config.planning_group.empty() ||
    !manipulation::validation::is_canonical_relative_frame_id(config.end_effector_link))
  {
    throw std::invalid_argument(
            "MoveIt planning endpoint/group must be non-empty and end-effector link canonical");
  }
  if (config.action_poll_period <= std::chrono::steady_clock::duration::zero() ||
    !finite_scaling(config.velocity_scaling) ||
    !finite_scaling(config.acceleration_scaling) ||
    !finite_positive(config.position_tolerance) ||
    !finite_positive(config.orientation_tolerance) ||
    !finite_positive(config.joint_tolerance))
  {
    throw std::invalid_argument(
            "MoveIt planning poll period, scaling, and tolerances must be valid and positive");
  }
}

class MoveItArmPlanningAdapter::Impl final
{
public:
  Impl(rclcpp::Node & node, MoveItArmPlanningAdapterConfig config)
  : config_(std::move(config)),
    context_(node.get_node_base_interface()->get_context()),
    callback_group_(node.create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    validateMoveItArmPlanningAdapterConfig(config_);
    client_ = rclcpp_action::create_client<MoveGroup>(
      &node, config_.move_group_action, callback_group_);
  }

  ~Impl()
  {
    request_stop();
  }

  ArmPlanningReadiness cached_readiness() const
  {
    if (active_slot_.dispatch_unknown()) {
      return {
        false,
        "previous MoveGroup dispatch is unknown; wait for a terminal result or restart adapter"};
    }
    if (active_slot_.busy()) {
      return {false, "previous MoveGroup planning request has not reached a terminal result"};
    }
    if (!client_->action_server_is_ready()) {
      return {false, "MoveGroup planning action is unavailable"};
    }
    return {true, "MoveGroup plan-only endpoint is ready"};
  }

  ArmPlanningResult plan_once(
    const ArmPlanningRequest & request,
    const PlanningStopToken & stop_token)
  {
    const auto dispatch_generation =
      stop_generation_.load(std::memory_order_acquire);
    if (stop_token.stop_requested()) {
      return stopped_result(stop_token, "MoveGroup planning was canceled before validation");
    }
    if (active_slot_.dispatch_unknown() || active_slot_.busy()) {
      return ArmPlanningResult::failure(
        PlanningOutcome::kPlannerUnavailable,
        "MoveGroup plan-only adapter still owns a previous request");
    }

    std::string error;
    std::optional<MoveGroup::Goal> goal;
    try {
      goal = make_goal(config_, request, error);
    } catch (const std::exception & exception) {
      return ArmPlanningResult::failure(
        PlanningOutcome::kInvalidRequest,
        std::string{"MoveGroup request construction failed: "} + exception.what());
    }
    if (!goal) {
      return ArmPlanningResult::failure(
        PlanningOutcome::kInvalidRequest, std::move(error));
    }

    // Endpoint discovery is a non-physical wait.  The implementation-owned
    // MoveIt planning budget starts inside MoveGroup after goal acceptance;
    // transient ROS discovery delay must therefore not reject the transfer.
    const auto call = detail::call_action_until<MoveGroup>(
      client_, *goal, detail::unbounded_action_deadline(),
      config_.action_poll_period, context_, stop_token,
      [this, dispatch_generation]() {
        return stop_generation_.load(std::memory_order_acquire) !=
               dispatch_generation;
      },
      active_slot_,
      dispatch_mutex_, true);
    if (call.status != detail::ActionCallStatus::kResultReceived || !call.result) {
      return transport_failure(call.status, stop_token, call.detail);
    }
    const auto & wrapped = *call.result;
    if (!wrapped.result) {
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError, "MoveGroup returned a null result payload");
    }
    if (wrapped.code == ResultCode::CANCELED) {
      return stopped_result(stop_token, "MoveGroup planning goal was canceled");
    }

    const auto code = wrapped.result->error_code.val;
    if (wrapped.code != ResultCode::SUCCEEDED || code != MoveItErrorCodes::SUCCESS) {
      return moveit_failure(code, stop_token, result_detail(*wrapped.result));
    }
    const auto & trajectory = wrapped.result->planned_trajectory;
    if (trajectory.joint_trajectory.points.empty() &&
      trajectory.multi_dof_joint_trajectory.points.empty())
    {
      return ArmPlanningResult::failure(
        PlanningOutcome::kInternalError,
        "MoveGroup reported success with an empty planned trajectory", code);
    }
    return ArmPlanningResult::success(
      trajectory, "MoveGroup returned a valid plan-only trajectory");
  }

  void request_stop() noexcept
  {
    const std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
    active_slot_.request_cancel();
  }

private:
  const MoveItArmPlanningAdapterConfig config_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<MoveGroup>::SharedPtr client_;
  std::mutex dispatch_mutex_;
  detail::ActiveActionSlot active_slot_;
  std::atomic<std::uint64_t> stop_generation_{0U};
};

MoveItArmPlanningAdapter::MoveItArmPlanningAdapter(
  rclcpp::Node & node,
  MoveItArmPlanningAdapterConfig config)
: impl_(std::make_unique<Impl>(node, std::move(config)))
{
}

MoveItArmPlanningAdapter::~MoveItArmPlanningAdapter() = default;

ArmPlanningReadiness MoveItArmPlanningAdapter::cached_readiness() const
{
  return impl_->cached_readiness();
}

ArmPlanningResult MoveItArmPlanningAdapter::plan_once(
  const ArmPlanningRequest & request,
  const PlanningStopToken & stop_token)
{
  return impl_->plan_once(request, stop_token);
}

void MoveItArmPlanningAdapter::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::moveit
