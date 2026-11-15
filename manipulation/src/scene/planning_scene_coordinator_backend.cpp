// Responsibility: Executes bounded MoveIt PlanningScene backend reads and writes for the
//   coordinator.

#include "planning_scene_coordinator_node.hpp"

#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace manipulation::scene
{

PlanningSceneCoordinatorNode::BackendApplyAttempt
PlanningSceneCoordinatorNode::apply_to_moveit(
  const moveit_msgs::msg::PlanningScene & scene_diff,
  const std::string & operation_ticket)
{
  std::lock_guard<std::mutex> backend_lock(backend_call_mutex_);
  if (!expected_scene_ || !expected_scene_digest_) {
    return {
      true,
      make_backend_operation_result(
        BackendCallOutcome::kInternalError,
        "backend apply attempted without an authoritative scene baseline"),
      false};
  }
  const std::string baseline_digest = *expected_scene_digest_;

  auto transition = evolve_authoritative_scene(
    *expected_scene_, scene_diff, planning_frame_);
  if (!transition.valid) {
    return {
      true,
      {false, transition.message, RegistryError::kInvalidRequest},
      true};
  }

  if (const auto failure = express_transition_in_backend_frame(transition)) {
    return {true, *failure, true};
  }
  const auto expected_success_digest =
    digest_authoritative_scene(transition.expected_scene);

  const auto now = std::chrono::steady_clock::now();
  if (backend_service_timeout_ > std::chrono::steady_clock::time_point::max() - now) {
    return {
      true,
      make_backend_operation_result(
        BackendCallOutcome::kInternalError,
        "backend timeout exceeds steady-clock deadline range"),
      false};
  }
  const auto deadline = now + backend_service_timeout_;
  if (!backend_client_->service_is_ready()) {
    return {
      true,
      {false, "MoveIt apply service disappeared before dispatch",
        RegistryError::kDependencyUnavailable},
      true};
  }

  const auto remaining = deadline - std::chrono::steady_clock::now();
  if (remaining <= std::chrono::steady_clock::duration::zero()) {
    return {
      true,
      make_backend_operation_result(
        BackendCallOutcome::kTimeout,
        "PlanningScene pre-dispatch work exhausted the backend deadline"),
      true};
  }

  auto request = std::make_shared<ApplyPlanningScene::Request>();
  request->scene = transition.backend_diff;
  auto future = backend_client_->async_send_request(request);
  if (future.wait_for(remaining) != std::future_status::ready) {
    // Do not remove the pending request and do not complete the registry ticket:
    // removing a client future cannot cancel a server-side scene mutation.
    pending_operation_.emplace(PendingBackendOperation{
        std::move(future), operation_ticket, baseline_digest,
        transition.expected_scene, expected_success_digest,
        std::chrono::steady_clock::now()});
    require_reconciliation(
      RegistryError::kTimeout,
      "MoveIt apply response timed out; backend outcome is ambiguous");
    RCLCPP_WARN(
      get_logger(),
      "PlanningScene apply timed out after dispatch; tracking late response for ticket=%s",
      operation_ticket.c_str());
    return {
      false,
      make_backend_operation_result(
        BackendCallOutcome::kTimeout,
        "MoveIt apply response timed out; operation remains pending"),
      false};
  }

  BackendOperationResult result;
  try {
    const auto response = future.get();
    result = response && response->success ?
      make_backend_operation_result(
      BackendCallOutcome::kSuccess, "PlanningScene diff applied") :
      make_backend_operation_result(
      BackendCallOutcome::kRejected, "MoveIt rejected the PlanningScene diff");
  } catch (const std::exception & error) {
    result = make_backend_operation_result(BackendCallOutcome::kInternalError, error.what());
  }

  const auto readback = read_authoritative_scene_locked();
  const bool reconciliation_confirmed = reconcile_backend_result_locked(
    result, readback, transition.expected_scene, expected_success_digest,
    baseline_digest, "backend completion");
  return {true, std::move(result), reconciliation_confirmed};
}

bool PlanningSceneCoordinatorNode::reconcile_backend_result_locked(
  BackendOperationResult & result,
  const SceneReadbackResult & readback,
  const moveit_msgs::msg::PlanningScene & expected_success_scene,
  const std::string & expected_success_digest,
  const std::string & baseline_digest,
  const std::string & context)
{
  if (result.success) {
    // A successful response commits the intended state as the only automatic
    // recovery target. An unrelated readback is never adopted implicitly.
    expected_scene_ = expected_success_scene;
    expected_scene_digest_ = expected_success_digest;
  }
  if (!readback.success) {
    require_reconciliation(
      readback.error, context + " could not read authoritative scene: " + readback.message);
    return false;
  }
  if (readback.digest == expected_success_digest) {
    if (!result.success) {
      result = make_backend_operation_result(
        BackendCallOutcome::kSuccess,
        context + " reached the requested state despite a negative backend response");
    }
    accept_reconciled_scene(readback.scene, readback.digest);
    return true;
  }
  if (!result.success && readback.digest == baseline_digest) {
    accept_reconciled_scene(readback.scene, readback.digest);
    return true;
  }

  require_reconciliation(
    RegistryError::kReconciliationRequired,
    context + " does not match either the baseline or deterministic expected state");
  return false;
}

void PlanningSceneCoordinatorNode::settle_pending_operation_locked()
{
  if (!pending_operation_ ||
    pending_operation_->future.wait_for(std::chrono::seconds::zero()) !=
    std::future_status::ready)
  {
    return;
  }

  BackendOperationResult result;
  try {
    const auto response = pending_operation_->future.get();
    result = response && response->success ?
      make_backend_operation_result(
      BackendCallOutcome::kSuccess, "late PlanningScene apply response confirmed success") :
      make_backend_operation_result(
      BackendCallOutcome::kRejected, "late PlanningScene apply response reported failure");
  } catch (const std::exception & error) {
    result = make_backend_operation_result(BackendCallOutcome::kInternalError, error.what());
  }

  const auto ticket = pending_operation_->operation_ticket;
  const auto baseline = pending_operation_->baseline_digest;
  const auto expected_success_scene = pending_operation_->expected_success_scene;
  const auto expected_success_digest = pending_operation_->expected_success_digest;
  pending_operation_.reset();
  RCLCPP_INFO(
    get_logger(), "Received late PlanningScene backend response for ticket=%s",
    ticket.c_str());
  const auto readback = read_authoritative_scene_locked();
  (void)reconcile_backend_result_locked(
    result, readback, expected_success_scene, expected_success_digest, baseline,
    "late backend completion");

  const auto status = registry_->complete_operation(ticket, result);
  expire_leases();
  record_backend_status(status);
}

void PlanningSceneCoordinatorNode::settle_pending_operation()
{
  std::lock_guard<std::mutex> backend_lock(backend_call_mutex_);
  settle_pending_operation_locked();
}

}  // namespace manipulation::scene
