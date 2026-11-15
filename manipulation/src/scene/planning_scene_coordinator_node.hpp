// Responsibility: Owns authoritative scene revision, lease registry, backend clients and
//   diagnostics.

#ifndef SCENE__PLANNING_SCENE_COORDINATOR_NODE_HPP_
#define SCENE__PLANNING_SCENE_COORDINATOR_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <manipulation_msgs/srv/acquire_planning_scene_lease.hpp>
#include <manipulation_msgs/srv/apply_planning_scene_diff.hpp>
#include <manipulation_msgs/srv/get_planning_scene_state.hpp>
#include <manipulation_msgs/srv/reconcile_planning_scene.hpp>
#include <manipulation_msgs/srv/release_planning_scene_lease.hpp>
#include <manipulation_msgs/srv/renew_planning_scene_lease.hpp>
#include <manipulation_msgs/srv/validate_planning_scene_lease.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/scene_lease_registry.hpp"

namespace manipulation::scene
{

/**
 * Thin ROS adapter that serializes validated scene mutations through MoveIt.
 *
 * The node owns ROS parameters, services, timers, diagnostics and bounded
 * MoveIt apply/readback calls. A timed-out apply remains tracked because client
 * cancellation cannot undo a server-side mutation. Periodic and pre-side-effect
 * readback compares a deterministic complete world/attached/ACM snapshot.
 *
 * Lease, revision and idempotency policy remains in `SceneLeaseRegistry`; diff
 * coverage validation, canonical digesting and conversions remain in
 * independently testable helpers. Public world-object requests use
 * `planning_frame_`; only the ROS edge converts them to MoveIt's verified model
 * frame. The conversion samples one latest transform at apply time because the
 * supported `world` to fixed-root relationship is static; dynamic relationships
 * are outside this contract.
 *
 * The node is designed for a MultiThreadedExecutor. Callback groups keep
 * metadata responsive while serializing scene I/O; `backend_call_mutex_` is
 * never nested with the registry lock. Service and lease deadlines use steady
 * time. A timed-out backend apply is retained as pending and forces
 * reconciliation instead of claiming that no mutation occurred.
 */
class PlanningSceneCoordinatorNode final : public rclcpp::Node
{
public:
  explicit PlanningSceneCoordinatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using AcquireLease = manipulation_msgs::srv::AcquirePlanningSceneLease;
  using RenewLease = manipulation_msgs::srv::RenewPlanningSceneLease;
  using ReleaseLease = manipulation_msgs::srv::ReleasePlanningSceneLease;
  using ValidateLease = manipulation_msgs::srv::ValidatePlanningSceneLease;
  using ApplySceneDiff = manipulation_msgs::srv::ApplyPlanningSceneDiff;
  using GetSceneState = manipulation_msgs::srv::GetPlanningSceneState;
  using ReconcileScene = manipulation_msgs::srv::ReconcilePlanningScene;
  using ApplyPlanningScene = moveit_msgs::srv::ApplyPlanningScene;
  using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

  struct BackendApplyAttempt
  {
    // False means the request crossed the process boundary but its response is
    // still unknown. The registry ticket is then owned by pending_operation_.
    bool terminal{true};
    BackendOperationResult result;
    // True only after a post-response authoritative readback proves the new or
    // unchanged baseline. A terminal backend response alone is insufficient.
    bool reconciliation_confirmed{false};
  };

  struct SceneReadbackResult
  {
    bool success{false};
    moveit_msgs::msg::PlanningScene scene;
    std::string digest;
    RegistryError error{RegistryError::kBackendFailed};
    std::string message;
  };

  struct PendingBackendOperation
  {
    rclcpp::Client<ApplyPlanningScene>::FutureAndRequestId future;
    std::string operation_ticket;
    std::string baseline_digest;
    moveit_msgs::msg::PlanningScene expected_success_scene;
    std::string expected_success_digest;
    std::chrono::steady_clock::time_point timed_out_at;
  };

  void handle_acquire(
    const AcquireLease::Request::SharedPtr request,
    AcquireLease::Response::SharedPtr response);

  void handle_renew(
    const RenewLease::Request::SharedPtr request,
    RenewLease::Response::SharedPtr response);

  void handle_release(
    const ReleaseLease::Request::SharedPtr request,
    ReleaseLease::Response::SharedPtr response);

  void handle_validate(
    const ValidateLease::Request::SharedPtr request,
    ValidateLease::Response::SharedPtr response);

  void handle_apply(
    const ApplySceneDiff::Request::SharedPtr request,
    ApplySceneDiff::Response::SharedPtr response);

  void handle_get_state(
    const GetSceneState::Request::SharedPtr request,
    GetSceneState::Response::SharedPtr response);

  void handle_reconcile(
    const ReconcileScene::Request::SharedPtr request,
    ReconcileScene::Response::SharedPtr response);

  BackendApplyAttempt apply_to_moveit(
    const moveit_msgs::msg::PlanningScene & scene_diff,
    const std::string & operation_ticket);

  RegistryStatus validate_scene_dependencies(
    const moveit_msgs::msg::PlanningScene & scene_diff) const;

  std::optional<BackendOperationResult> express_transition_in_backend_frame(
    SceneEvolutionResult & transition);

  SceneReadbackResult read_authoritative_scene_locked();

  bool reconcile_backend_result_locked(
    BackendOperationResult & result,
    const SceneReadbackResult & readback,
    const moveit_msgs::msg::PlanningScene & expected_success_scene,
    const std::string & expected_success_digest,
    const std::string & baseline_digest,
    const std::string & context);

  RegistryStatus ensure_reconciled();

  void settle_pending_operation();

  void settle_pending_operation_locked();

  void maintain_consistency();

  void update_backend_readiness();

  [[nodiscard]] bool backend_services_ready() const;

  [[nodiscard]] bool backend_model_frame_ready() const;

  [[nodiscard]] bool world_transform_ready() const;

  void handle_robot_description(const std_msgs::msg::String::SharedPtr message);

  void require_reconciliation(RegistryError error, std::string message);

  void accept_reconciled_scene(
    moveit_msgs::msg::PlanningScene scene,
    std::string digest);

  void expire_leases();

  void record_backend_status(const RegistryStatus & status);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

  RegistryStatus invalid_request_status(std::string message) const;

  std::string backend_service_name_;
  std::string readback_service_name_;
  // Public mutations use planning_frame_; MoveIt canonicalizes world objects
  // into backend_planning_frame_. The TF adapter is confined to the ROS edge.
  std::string planning_frame_;
  std::string backend_planning_frame_;
  std::string robot_description_topic_;
  bool verify_fixed_root_model_frame_{true};
  std::chrono::steady_clock::duration backend_service_timeout_{};
  std::chrono::steady_clock::duration readback_service_timeout_{};
  std::unique_ptr<SceneLeaseRegistry> registry_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr scene_io_group_;
  rclcpp::CallbackGroup::SharedPtr backend_group_;
  rclcpp::CallbackGroup::SharedPtr maintenance_group_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr backend_client_;
  rclcpp::Client<GetPlanningScene>::SharedPtr readback_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::Service<AcquireLease>::SharedPtr acquire_service_;
  rclcpp::Service<RenewLease>::SharedPtr renew_service_;
  rclcpp::Service<ReleaseLease>::SharedPtr release_service_;
  rclcpp::Service<ValidateLease>::SharedPtr validate_service_;
  rclcpp::Service<ApplySceneDiff>::SharedPtr apply_service_;
  rclcpp::Service<GetSceneState>::SharedPtr state_service_;
  rclcpp::Service<ReconcileScene>::SharedPtr reconcile_service_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  rclcpp::TimerBase::SharedPtr reconciliation_timer_;
  diagnostic_updater::Updater diagnostics_;

  // Serializes backend apply/readback sequences. No registry mutex is held
  // while this mutex is held; the registry ticket preserves logical ownership.
  mutable std::mutex backend_call_mutex_;
  std::optional<moveit_msgs::msg::PlanningScene> expected_scene_;
  std::optional<std::string> expected_scene_digest_;
  std::optional<PendingBackendOperation> pending_operation_;

  mutable std::mutex model_frame_mutex_;
  bool backend_model_frame_verified_{false};
  std::string observed_model_frame_;
  std::string model_frame_verification_message_{
    "awaiting fixed-root robot_description"};

  mutable std::mutex health_mutex_;
  RegistryError last_backend_error_{RegistryError::kNone};
  std::string last_backend_message_{"startup reconciliation has not completed"};
  bool backend_service_ready_{false};
  bool reconciliation_required_{true};
  std::uint64_t last_reported_expiry_count_{0};
  std::uint64_t unreconciled_expiry_count_{0};
};

}  // namespace manipulation::scene

#endif  // SCENE__PLANNING_SCENE_COORDINATOR_NODE_HPP_
