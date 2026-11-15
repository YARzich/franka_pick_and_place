// Responsibility: Own the complete Franka V2 transfer workflow composition.

#ifndef WORKFLOW_V2__COMPOSITION_HPP_
#define WORKFLOW_V2__COMPOSITION_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation/workflow/resolved_transfer_workflow.hpp"
#include "manipulation/workflow/strategy_registry.hpp"
#include "manipulation/workflow/transfer_object_sequence_v2_action_server.hpp"
#include "manipulation/workflow/wait_lifecycle_logger.hpp"
#include "manipulation_moveit/arm_planning_port.hpp"
#include "manipulation_moveit/moveit_arm_planning_adapter.hpp"
#include "manipulation_moveit/pick_lift_place_strategy.hpp"
#include "manipulation_moveit/ros_pick_lift_place_health_port.hpp"
#include "manipulation_moveit/ros_v2_physical_ports.hpp"
#include "manipulation_moveit/ros_v2_scene_adapter.hpp"

#include "workflow_v2/isaac_physical_scene_port.hpp"
#include "workflow_v2/process_config.hpp"
#include "workflow_v2/provider_runtime.hpp"

namespace franka_motion::workflow_v2
{

namespace detail
{

/** Stable log label for an optional terminal wait outcome. */
[[nodiscard]] const char * wait_lifecycle_outcome_name(
  const std::optional<manipulation::workflow::WaitLifecycleOutcome> & outcome) noexcept;

/** Exact one-to-one mapping from validated process data to concrete adapters. */
struct CompositionBindings
{
  manipulation::moveit::RosV2SceneAdapterConfig scene;
  IsaacPhysicalScenePortConfig physical_scene;
  manipulation::moveit::MoveItArmPlanningAdapterConfig planning;
  manipulation::moveit::PickLiftPlaceStrategyConfig strategy;
  manipulation::moveit::RosTrajectoryExecutionPortConfig trajectory_execution;
  manipulation::moveit::RosStrategyGripperPortConfig gripper;
  manipulation::moveit::RosPickLiftPlaceHealthPortConfig health;
  manipulation::workflow::ResolvedTransferWorkflowOptions workflow;
  manipulation::workflow::TransferObjectSequenceV2ActionServerConfig action_server;
};

/** Build and validate composition inputs without constructing ROS entities. */
[[nodiscard]] CompositionBindings make_composition_bindings(
  const LoadedV2ProcessConfig & config,
  std::shared_ptr<manipulation::workflow::WaitLifecycleLogger> wait_logger);

}  // namespace detail

/**
 * One process-owned graph from the sole public V2 action to physical ports.
 *
 * Member order is the ownership contract: the action facade and workflow die
 * before every borrowed strategy/provider/backend. The node is retained only
 * to keep callback resources alive and must be removed from its executor before
 * this object is destroyed.
 */
class FrankaTransferWorkflowV2 final
{
public:
  FrankaTransferWorkflowV2(
    rclcpp::Node::SharedPtr node,
    LoadedV2ProcessConfig config);
  ~FrankaTransferWorkflowV2();

  FrankaTransferWorkflowV2(const FrankaTransferWorkflowV2 &) = delete;
  FrankaTransferWorkflowV2 & operator=(const FrankaTransferWorkflowV2 &) = delete;
  FrankaTransferWorkflowV2(FrankaTransferWorkflowV2 &&) = delete;
  FrankaTransferWorkflowV2 & operator=(FrankaTransferWorkflowV2 &&) = delete;

private:
  rclcpp::Node::SharedPtr node_;
  LoadedV2ProcessConfig config_;
  detail::CompositionBindings bindings_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  ProviderRuntime providers_;
  manipulation::moveit::RosV2SceneAdapter scene_;
  IsaacPhysicalScenePort strategy_scene_;
  manipulation::moveit::MoveItArmPlanningAdapter planning_backend_;
  manipulation::moveit::ArmPlanningPort planning_;
  manipulation::moveit::RosTrajectoryExecutionPort trajectory_execution_;
  manipulation::moveit::RosStrategyGripperPort gripper_;
  manipulation::moveit::RosPickLiftPlaceHealthPort health_;
  manipulation::workflow::StrategyRegistry strategies_;
  manipulation::workflow::ResolvedTransferWorkflow workflow_;
  std::atomic<std::uint64_t> goal_sequence_{0U};
  manipulation::workflow::TransferObjectSequenceV2ActionServer action_server_;
};

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__COMPOSITION_HPP_
