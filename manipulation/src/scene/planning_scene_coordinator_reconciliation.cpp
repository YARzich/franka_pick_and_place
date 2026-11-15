// Responsibility: Reconciles coordinator state with MoveIt and quarantines integrity mismatches.

#include "planning_scene_coordinator_node.hpp"

#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <moveit_msgs/msg/planning_scene_components.hpp>

namespace manipulation::scene
{

bool PlanningSceneCoordinatorNode::backend_services_ready() const
{
  return backend_client_->service_is_ready() && readback_client_->service_is_ready() &&
         backend_model_frame_ready();
}

bool PlanningSceneCoordinatorNode::world_transform_ready() const
{
  return planning_frame_ == backend_planning_frame_ ||
         tf_buffer_->canTransform(
    backend_planning_frame_, planning_frame_, tf2::TimePointZero,
    tf2::durationFromSec(0.0));
}

void PlanningSceneCoordinatorNode::require_reconciliation(
  const RegistryError error,
  std::string message)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  reconciliation_required_ = true;
  last_backend_error_ = error;
  last_backend_message_ = std::move(message);
}

void PlanningSceneCoordinatorNode::accept_reconciled_scene(
  moveit_msgs::msg::PlanningScene scene,
  std::string digest)
{
  // Caller holds backend_call_mutex_, which serializes this baseline with every
  // apply and readback. Health fields have their own short critical section.
  expected_scene_digest_ = std::move(digest);
  expected_scene_ = std::move(scene);
  const bool ready = backend_services_ready();
  const auto snapshot = registry_->snapshot();
  const bool quarantine_active = !snapshot.quarantined_resource_keys.empty();
  std::lock_guard<std::mutex> lock(health_mutex_);
  backend_service_ready_ = ready;
  reconciliation_required_ = !ready || quarantine_active;
  if (quarantine_active) {
    last_backend_error_ = RegistryError::kResourceQuarantined;
    last_backend_message_ =
      "authoritative PlanningScene readback cannot clear quarantined resource ownership";
  } else if (ready) {
    last_backend_error_ = RegistryError::kNone;
    last_backend_message_ = "authoritative PlanningScene readback matches coordinator state";
  } else {
    last_backend_error_ = RegistryError::kBackendFailed;
    last_backend_message_ =
      "scene readback succeeded but a backend service or model-frame check is unavailable";
  }
  if (ready && !quarantine_active) {
    unreconciled_expiry_count_ = 0;
  }
}

void PlanningSceneCoordinatorNode::update_backend_readiness()
{
  const bool ready = backend_services_ready();
  bool was_ready = false;
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    was_ready = backend_service_ready_;
    backend_service_ready_ = ready;
  }
  if (!ready) {
    require_reconciliation(
      RegistryError::kBackendFailed,
      was_ready ?
      "PlanningScene backend dependency disappeared; readback is required after recovery" :
      "MoveIt apply/readback service or fixed-root model-frame verification is unavailable");
  }
}

PlanningSceneCoordinatorNode::SceneReadbackResult
PlanningSceneCoordinatorNode::read_authoritative_scene_locked()
{
  const auto now = std::chrono::steady_clock::now();
  if (readback_service_timeout_ > std::chrono::steady_clock::time_point::max() - now) {
    return {
      false, {}, {}, RegistryError::kInternalError,
      "readback timeout exceeds steady-clock deadline range"};
  }
  const auto deadline = now + readback_service_timeout_;

  if (!readback_client_->service_is_ready() &&
    !readback_client_->wait_for_service(readback_service_timeout_))
  {
    return {
      false, {}, {}, RegistryError::kBackendFailed,
      "MoveIt get_planning_scene service is unavailable"};
  }
  const auto remaining = deadline - std::chrono::steady_clock::now();
  if (remaining <= std::chrono::steady_clock::duration::zero()) {
    return {
      false, {}, {}, RegistryError::kTimeout,
      "MoveIt readback service became ready after the configured deadline"};
  }

  using Components = moveit_msgs::msg::PlanningSceneComponents;
  auto request = std::make_shared<GetPlanningScene::Request>();
  request->components.components =
    Components::WORLD_OBJECT_GEOMETRY |
    Components::ROBOT_STATE_ATTACHED_OBJECTS |
    Components::ALLOWED_COLLISION_MATRIX;
  auto future = readback_client_->async_send_request(request);
  if (future.wait_for(remaining) != std::future_status::ready) {
    // Readback has no side effect, so abandoning its late response is safe.
    readback_client_->remove_pending_request(future);
    return {
      false, {}, {}, RegistryError::kTimeout,
      "MoveIt get_planning_scene response timed out"};
  }

  try {
    const auto response = future.get();
    if (!response) {
      return {
        false, {}, {}, RegistryError::kBackendFailed,
        "MoveIt returned an empty PlanningScene readback response"};
    }
    for (const auto & object : response->scene.world.collision_objects) {
      if (object.header.frame_id != backend_planning_frame_) {
        return {
          false, {}, {}, RegistryError::kBackendFailed,
          "authoritative world object '" + object.id + "' uses frame '" +
          object.header.frame_id + "' instead of backend_planning_frame '" +
          backend_planning_frame_ + "'"};
      }
    }
    auto scene = canonical_authoritative_scene(response->scene);
    auto digest = digest_authoritative_scene(scene);
    return {
      true, std::move(scene), std::move(digest), RegistryError::kNone,
      "authoritative PlanningScene readback received"};
  } catch (const std::exception & error) {
    return {false, {}, {}, RegistryError::kInternalError, error.what()};
  }
}

RegistryStatus PlanningSceneCoordinatorNode::ensure_reconciled()
{
  std::lock_guard<std::mutex> backend_lock(backend_call_mutex_);
  const auto snapshot = registry_->snapshot();
  if (pending_operation_) {
    return {
      false, RegistryError::kOperationInProgress, snapshot.revision,
      "an ambiguous PlanningScene apply is awaiting its authoritative response"};
  }
  if (!backend_services_ready()) {
    require_reconciliation(
      RegistryError::kBackendFailed,
      "MoveIt apply/readback service or fixed-root model-frame verification is unavailable");
    return {
      false, RegistryError::kReconciliationRequired, snapshot.revision,
      "authoritative PlanningScene dependencies are unavailable"};
  }

  const auto readback = read_authoritative_scene_locked();
  if (!readback.success) {
    require_reconciliation(readback.error, readback.message);
    return {
      false, RegistryError::kReconciliationRequired, snapshot.revision,
      "PlanningScene readback failed: " + readback.message};
  }
  const auto after_readback = registry_->snapshot();
  if (!after_readback.quarantined_resource_keys.empty()) {
    require_reconciliation(
      RegistryError::kResourceQuarantined,
      "authoritative PlanningScene readback cannot prove the physical disposition of "
      "quarantined resources");
    return {
      false, RegistryError::kResourceQuarantined, after_readback.revision,
      "PlanningScene resources remain quarantined until explicit recovery acknowledgement"};
  }
  if (expected_scene_digest_ && *expected_scene_digest_ != readback.digest) {
    require_reconciliation(
      RegistryError::kBackendFailed,
      "authoritative PlanningScene differs from the last confirmed coordinator state");
    return {
      false, RegistryError::kReconciliationRequired, snapshot.revision,
      "authoritative PlanningScene readback mismatch"};
  }

  accept_reconciled_scene(readback.scene, readback.digest);
  return {
    true, RegistryError::kNone, snapshot.revision,
    "PlanningScene coordinator and MoveIt are reconciled"};
}

void PlanningSceneCoordinatorNode::maintain_consistency()
{
  update_backend_readiness();
  settle_pending_operation();
  (void)ensure_reconciled();
}

}  // namespace manipulation::scene
