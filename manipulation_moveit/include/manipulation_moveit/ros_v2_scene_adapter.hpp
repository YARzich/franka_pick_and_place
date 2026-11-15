// Responsibility: Declares the production ROS scene boundary for the resolved V2 workflow.

#ifndef MANIPULATION_MOVEIT__ROS_V2_SCENE_ADAPTER_HPP_
#define MANIPULATION_MOVEIT__ROS_V2_SCENE_ADAPTER_HPP_

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "moveit/robot_model/robot_model.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation/workflow/object_geometry_conversion.hpp"
#include "manipulation/workflow/transfer_scene_workflow_port.hpp"

#include "manipulation_moveit/arm_planning_port.hpp"
#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace manipulation::moveit
{

/** Deployment identity and robot-owned scene resources; contains no timing policy. */
struct RosV2SceneAdapterConfig
{
  std::string owner_id{"transfer_object_sequence_v2"};
  std::string attachment_link;
  std::vector<std::string> touch_links;

  std::string acquire_service{"/planning_scene_coordinator/acquire_lease"};
  std::string renew_service{"/planning_scene_coordinator/renew_lease"};
  std::string release_service{"/planning_scene_coordinator/release_lease"};
  std::string validate_service{"/planning_scene_coordinator/validate_lease"};
  std::string state_service{"/planning_scene_coordinator/get_state"};
  std::string apply_diff_service{"/planning_scene_coordinator/apply_scene_diff"};
  std::string get_planning_scene_service{"/get_planning_scene"};
};

/** Fail fast for malformed endpoint, owner, attachment-link, or touch-link data. */
void validateRosV2SceneAdapterConfig(const RosV2SceneAdapterConfig & config);

/** Production package/resource mesh loader used by catalog geometry conversion. */
class GeometricShapesMeshResourceLoader final
  : public manipulation::workflow::MeshResourceLoader
{
public:
  [[nodiscard]] shape_msgs::msg::Mesh load(
    std::string_view resource_uri,
    const std::array<double, 3> & scale) const override;
};

/**
 * Single production scene adapter shared by resolution, planning, and execution.
 *
 * Every authoritative mutation is sent only through ApplyPlanningSceneDiff.
 * Full snapshots are accepted only when coordinator state is stable before and
 * after GetPlanningScene and lease validation proves the same session/revision.
 * ROS endpoint and response waits have no semantic timeout: they end on a
 * response, typed final error, cooperative stop, or ROS shutdown. The lease TTL
 * and wait slice are private implementation constants and are not YAML policy.
 * The supplied node, and an injected mesh loader when that overload is used,
 * must outlive the adapter and every active call.
 */
class RosV2SceneAdapter final
  : public manipulation::workflow::TransferSceneWorkflowPort,
  public IPickLiftPlaceScenePort,
  public IPlanningSceneRevisionHook
{
public:
  RosV2SceneAdapter(
    rclcpp::Node & node,
    ::moveit::core::RobotModelConstPtr robot_model,
    RosV2SceneAdapterConfig config);

  RosV2SceneAdapter(
    rclcpp::Node & node,
    ::moveit::core::RobotModelConstPtr robot_model,
    RosV2SceneAdapterConfig config,
    const manipulation::workflow::MeshResourceLoader & mesh_loader);

  ~RosV2SceneAdapter() override;

  RosV2SceneAdapter(const RosV2SceneAdapter &) = delete;
  RosV2SceneAdapter & operator=(const RosV2SceneAdapter &) = delete;
  RosV2SceneAdapter(RosV2SceneAdapter &&) = delete;
  RosV2SceneAdapter & operator=(RosV2SceneAdapter &&) = delete;

  /** Cheap endpoint evidence for the composition-owned production health gate. */
  [[nodiscard]] PickLiftPlaceReadiness cached_readiness() const;

  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult acquire(
    const std::vector<manipulation::workflow::ObjectInstance> & instances,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] manipulation::workflow::PlanningSceneAccessProofResult renew(
    const manipulation::workflow::PlanningSceneAccessProof & access_proof,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult stage_observation(
    const manipulation::workflow::ObjectInstance & instance,
    const manipulation::workflow::ObjectProfile & profile,
    const manipulation::workflow::ObjectObservation & observation,
    const manipulation::workflow::PlanningSceneAccessProof & access_proof,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult revalidate(
    const manipulation::workflow::PlanningSceneAccessProof & access_proof,
    manipulation::workflow::TransferExecutionContext & context) override;

  void release(
    const manipulation::workflow::PlanningSceneAccessProof & access_proof) noexcept override;

  void begin_goal() override;

  [[nodiscard]] CoordinatedSceneResult revalidate(
    const StrategySceneRevalidationRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] CoordinatedSceneResult attach(
    const StrategySceneAttachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] CoordinatedSceneResult detach(
    const StrategySceneDetachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) override;

  [[nodiscard]] SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken & stop_token) override;

  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<manipulation::workflow::MeshResourceLoader> owned_mesh_loader_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__ROS_V2_SCENE_ADAPTER_HPP_
