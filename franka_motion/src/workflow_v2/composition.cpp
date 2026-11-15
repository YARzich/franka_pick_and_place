// Responsibility: Wire the validated Franka V2 profile to generic workflow ports.

#include "workflow_v2/composition.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/workflow/resolved_transfer_workflow_action_adapter.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation/workflow/wait_lifecycle_logger.hpp"
#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

using manipulation::moveit::PickLiftPlaceReadiness;
using manipulation::workflow::TransferWaitingReason;
using manipulation::workflow::WaitLifecycleEvent;
using manipulation::workflow::WaitLifecycleLogger;
using manipulation::workflow::WaitLifecycleRecord;

rclcpp::Node::SharedPtr require_node(rclcpp::Node::SharedPtr node)
{
  if (!node) {
    throw std::invalid_argument("Franka V2 workflow requires a ROS node");
  }
  return node;
}

moveit::core::RobotModelConstPtr require_robot_model(
  const robot_model_loader::RobotModelLoader & loader)
{
  const auto model = loader.getModel();
  if (!model) {
    throw std::runtime_error("robot_description did not produce a MoveIt robot model");
  }
  return model;
}

manipulation::moveit::RosV2SceneAdapterConfig make_scene_config(
  const LoadedV2ProcessConfig & config)
{
  manipulation::moveit::RosV2SceneAdapterConfig result;
  result.attachment_link = config.robot.attachment_link;
  result.touch_links = config.robot.touch_links;
  result.acquire_service = config.deployment.scene_acquire_service;
  result.renew_service = config.deployment.scene_renew_service;
  result.release_service = config.deployment.scene_release_service;
  result.validate_service = config.deployment.scene_validate_service;
  result.state_service = config.deployment.scene_state_service;
  result.apply_diff_service = config.deployment.scene_apply_service;
  result.get_planning_scene_service = config.deployment.planning_scene_read_service;
  return result;
}

manipulation::moveit::MoveItArmPlanningAdapterConfig make_planning_config(
  const LoadedV2ProcessConfig & config)
{
  manipulation::moveit::MoveItArmPlanningAdapterConfig result;
  result.move_group_action = config.moveit.move_group_action;
  result.planning_group = config.robot.planning_group;
  result.end_effector_link = config.robot.end_effector_link;
  result.planning_pipeline = config.moveit.planning_pipeline;
  result.planner_id = config.moveit.planner_id;
  result.velocity_scaling = config.moveit.velocity_scaling;
  result.acceleration_scaling = config.moveit.acceleration_scaling;
  result.position_tolerance = config.moveit.position_tolerance;
  result.orientation_tolerance = config.moveit.orientation_tolerance;
  result.joint_tolerance = config.moveit.joint_tolerance;
  return result;
}

manipulation::moveit::PickLiftPlaceStrategyConfig make_strategy_config(
  const LoadedV2ProcessConfig & config)
{
  manipulation::moveit::PickLiftPlaceStrategyConfig result;
  result.robot.attachment_link = config.robot.attachment_link;
  result.robot.touch_links = config.robot.touch_links;
  result.robot.open_width_m = config.robot.open_width_m;
  result.robot.allow_grasp_stall = config.robot.allow_grasp_stall;
  result.robot.home_joint_names = config.robot.home_joint_names;
  result.robot.home_joint_positions = config.robot.home_joint_positions;
  result.task.lift_clearance_m = config.pick_lift_place.lift_clearance_m;
  result.task.preplace_clearance_m = config.pick_lift_place.preplace_clearance_m;
  result.task.retreat_clearance_m = config.pick_lift_place.retreat_clearance_m;
  result.task.release_settle = config.pick_lift_place.release_settle;
  result.arm_motion_timeout = config.physical_actions.arm_motion;
  result.gripper_command_timeout = config.physical_actions.gripper_command;
  return result;
}

PickLiftPlaceReadiness planning_readiness(
  const manipulation::moveit::ArmPlanningPort & planning)
{
  const auto readiness = planning.cached_readiness();
  return {readiness.ready, readiness.detail};
}

manipulation::moveit::RosPickLiftPlaceReadinessProbes make_health_probes(
  manipulation::moveit::ArmPlanningPort & planning,
  manipulation::moveit::RosTrajectoryExecutionPort & trajectory,
  manipulation::moveit::RosStrategyGripperPort & gripper,
  IsaacPhysicalScenePort & scene)
{
  return {
    [&planning]() {return planning_readiness(planning);},
    [&trajectory]() {return trajectory.cached_readiness();},
    [&gripper]() {return gripper.cached_readiness();},
    [&scene]() {return scene.cached_readiness();}};
}

const char * wait_event_name(const WaitLifecycleEvent event) noexcept
{
  switch (event) {
    case WaitLifecycleEvent::kEnter:
      return "enter";
    case WaitLifecycleEvent::kWaiting:
      return "waiting";
    case WaitLifecycleEvent::kExit:
      return "exit";
  }
  return "unknown";
}

const char * waiting_reason_name(const TransferWaitingReason reason) noexcept
{
  switch (reason) {
    case TransferWaitingReason::kNone:
      return "none";
    case TransferWaitingReason::kObservation:
      return "observation";
    case TransferWaitingReason::kTransform:
      return "transform";
    case TransferWaitingReason::kGraspGeneration:
      return "grasp_generation";
    case TransferWaitingReason::kSceneReadiness:
      return "scene_readiness";
    case TransferWaitingReason::kHealthReadiness:
      return "health_readiness";
    case TransferWaitingReason::kPlanning:
      return "planning";
    case TransferWaitingReason::kRosEndpoint:
      return "ros_endpoint";
  }
  return "unknown";
}

std::shared_ptr<WaitLifecycleLogger> make_wait_logger(const rclcpp::Logger & logger)
{
  return std::make_shared<WaitLifecycleLogger>(
    [logger](WaitLifecycleRecord record) {
      const auto elapsed = std::chrono::duration<double>(record.elapsed).count();
      RCLCPP_INFO(
        logger,
        "transfer_wait event=%s goal=%s step=%zu/%zu object=%s reason=%s "
        "elapsed_sec=%.3f outcome=%s detail=%s",
        wait_event_name(record.event), record.scope.goal_id.c_str(),
        record.scope.step_index + 1U, record.scope.step_count,
        record.scope.object_id.empty() ? "<goal>" : record.scope.object_id.c_str(),
        waiting_reason_name(record.scope.reason), elapsed,
        detail::wait_lifecycle_outcome_name(record.outcome),
        record.scope.diagnostic.c_str());
    });
}

manipulation::workflow::ResolvedTransferWorkflowOptions make_workflow_options(
  const LoadedV2ProcessConfig & config,
  std::shared_ptr<WaitLifecycleLogger> wait_logger)
{
  manipulation::workflow::ResolvedTransferWorkflowOptions result;
  result.validation.max_step_count = config.workflow.max_steps;
  result.validation.planning_frame = config.robot.planning_frame;
  result.validation.supported_recovery_policies = {
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_NONE,
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_SAFE_STOP,
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_RETREAT,
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_RETREAT_AND_HOME};
  result.wait_logger = std::move(wait_logger);
  return result;
}

manipulation::workflow::TransferObjectSequenceV2ActionServerConfig make_action_config(
  const LoadedV2ProcessConfig & config)
{
  return {
    config.deployment.sequence_action,
    config.deployment.sequence_pause_service};
}

}  // namespace

const char * detail::wait_lifecycle_outcome_name(
  const std::optional<manipulation::workflow::WaitLifecycleOutcome> & outcome) noexcept
{
  if (!outcome) {
    return "none";
  }
  using Outcome = manipulation::workflow::WaitLifecycleOutcome;
  switch (*outcome) {
    case Outcome::kReady:
      return "ready";
    case Outcome::kCanceled:
      return "canceled";
    case Outcome::kShutdown:
      return "shutdown";
    case Outcome::kError:
      return "error";
  }
  return "unknown";
}

detail::CompositionBindings detail::make_composition_bindings(
  const LoadedV2ProcessConfig & config,
  std::shared_ptr<WaitLifecycleLogger> wait_logger)
{
  if (!wait_logger) {
    throw std::invalid_argument("V2 composition requires a wait lifecycle logger");
  }

  CompositionBindings result;
  result.scene = make_scene_config(config);
  result.physical_scene.service_name =
    config.deployment.physical_attachment_service;
  result.planning = make_planning_config(config);
  result.strategy = make_strategy_config(config);
  result.trajectory_execution.action_name = config.moveit.execute_trajectory_action;
  result.gripper.action_name = config.robot.gripper_action;
  result.health.topic = config.deployment.health_topic;
  result.workflow = make_workflow_options(config, std::move(wait_logger));
  result.action_server = make_action_config(config);

  manipulation::moveit::validateRosV2SceneAdapterConfig(result.scene);
  validateIsaacPhysicalScenePortConfig(result.physical_scene);
  manipulation::moveit::validateMoveItArmPlanningAdapterConfig(result.planning);
  manipulation::moveit::validatePickLiftPlaceStrategyConfig(result.strategy);
  manipulation::moveit::validateRosTrajectoryExecutionPortConfig(
    result.trajectory_execution);
  manipulation::moveit::validateRosStrategyGripperPortConfig(result.gripper);
  return result;
}

FrankaTransferWorkflowV2::FrankaTransferWorkflowV2(
  rclcpp::Node::SharedPtr node,
  LoadedV2ProcessConfig config)
: node_(require_node(std::move(node))),
  config_(std::move(config)),
  bindings_(detail::make_composition_bindings(
      config_, make_wait_logger(node_->get_logger()))),
  robot_model_loader_(node_, "robot_description", true),
  providers_(node_, config_, require_robot_model(robot_model_loader_)),
  scene_(*node_, providers_.robot_model(), bindings_.scene),
  strategy_scene_(
    *node_, scene_, bindings_.physical_scene,
    [this]() {return scene_.cached_readiness();}),
  planning_backend_(*node_, bindings_.planning),
  planning_(planning_backend_, scene_),
  trajectory_execution_(*node_, bindings_.trajectory_execution),
  gripper_(*node_, bindings_.gripper),
  health_(
    *node_, bindings_.health,
    make_health_probes(planning_, trajectory_execution_, gripper_, strategy_scene_)),
  strategies_(manipulation::workflow::StrategyRegistry::production(
      std::make_unique<manipulation::moveit::PickLiftPlaceStrategy>(
        bindings_.strategy, planning_, trajectory_execution_, gripper_,
        strategy_scene_, health_))),
  workflow_(
    bindings_.workflow, providers_.object_catalog(),
    providers_.observation_provider(), providers_.grasp_provider(), scene_, strategies_,
    config_.workflow.strategy_id),
  action_server_(
    *node_, bindings_.action_server,
    manipulation::workflow::make_resolved_transfer_workflow_callbacks(
      workflow_, [this](const auto &) {
        return std::string {"v2-goal-"} +
               std::to_string(goal_sequence_.fetch_add(1U) + 1U);
      }))
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Franka V2 transfer workflow ready: action=%s strategy=%s observation=%s "
    "grasp=%s planning_pipeline=%s",
    config_.deployment.sequence_action.c_str(), config_.workflow.strategy_id.c_str(),
    config_.workflow.observation_provider_id.c_str(),
    config_.workflow.grasp_provider_id.c_str(),
    config_.moveit.planning_pipeline.empty() ? "<move_group_default>" :
    config_.moveit.planning_pipeline.c_str());
}

FrankaTransferWorkflowV2::~FrankaTransferWorkflowV2() = default;

}  // namespace franka_motion::workflow_v2
