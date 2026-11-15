// Responsibility: Serves scene state, lease acquisition, renewal and release requests.

#include "planning_scene_coordinator_node.hpp"

#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace manipulation::scene
{

void PlanningSceneCoordinatorNode::handle_get_state(
  const GetSceneState::Request::SharedPtr request,
  GetSceneState::Response::SharedPtr response)
{
  (void)request;
  expire_leases();
  update_backend_readiness();
  const auto snapshot = registry_->snapshot();
  auto state = to_interface_state(snapshot);
  if (!state) {
    response->result = to_interface_result(
      {false, RegistryError::kInternalError, snapshot.revision,
        "coordinator counters exceed interface capacity"});
    return;
  }

  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    state->backend_service_ready = backend_service_ready_;
    state->reconciliation_required = reconciliation_required_;
    state->last_backend_error_code = to_interface_error(last_backend_error_);
    state->expired_lease_count = snapshot.expired_lease_count;
    state->unreconciled_lease_expiry_count = unreconciled_expiry_count_;
  }
  response->state = *state;
  response->result = to_interface_result(
    {true, RegistryError::kNone, snapshot.revision, "coordinator state returned"});
}

void PlanningSceneCoordinatorNode::handle_reconcile(
  const ReconcileScene::Request::SharedPtr request,
  ReconcileScene::Response::SharedPtr response)
{
  if (!request->accept_authoritative_state) {
    response->result = to_interface_result(
      invalid_request_status("accept_authoritative_state must be explicitly enabled"));
    return;
  }
  if (request->quarantine_resolution !=
    ReconcileScene::Request::PRESERVE_QUARANTINE &&
    request->quarantine_resolution !=
    ReconcileScene::Request::ACCEPT_QUARANTINED_RESOURCES)
  {
    response->result = to_interface_result(
      invalid_request_status("quarantine_resolution uses an unknown value"));
    return;
  }

  std::lock_guard<std::mutex> backend_lock(backend_call_mutex_);
  expire_leases();
  const auto before = registry_->snapshot();
  if (request->expected_session_id != before.session_id) {
    response->result = to_interface_result(
      {false, RegistryError::kSessionMismatch, before.revision,
        "reconciliation request targets another coordinator session"});
    return;
  }
  if (request->expected_revision != before.revision) {
    response->result = to_interface_result(
      {false, RegistryError::kRevisionMismatch, before.revision,
        "reconciliation request uses a stale revision"});
    return;
  }
  if (!before.quarantined_resource_keys.empty() &&
    request->quarantine_resolution !=
    ReconcileScene::Request::ACCEPT_QUARANTINED_RESOURCES)
  {
    response->result = to_interface_result(
      {false, RegistryError::kResourceQuarantined, before.revision,
        "recovery must explicitly acknowledge quarantined resource disposition"});
    return;
  }
  if (pending_operation_ || before.operation_in_progress) {
    response->result = to_interface_result(
      {false, RegistryError::kOperationInProgress, before.revision,
        "cannot reconcile while an apply request may still mutate MoveIt"});
    return;
  }
  if (!backend_services_ready()) {
    require_reconciliation(
      RegistryError::kBackendFailed,
      "cannot reconcile while a MoveIt PlanningScene dependency is unavailable");
    response->result = to_interface_result(
      {false, RegistryError::kReconciliationRequired, before.revision,
        "authoritative PlanningScene dependencies are unavailable"});
    return;
  }

  const auto readback = read_authoritative_scene_locked();
  if (!readback.success) {
    require_reconciliation(readback.error, readback.message);
    response->result = to_interface_result(
      {false, RegistryError::kReconciliationRequired, before.revision,
        "authoritative PlanningScene readback failed: " + readback.message});
    return;
  }

  std::string new_session_id;
  try {
    new_session_id = generate_session_id();
  } catch (const std::exception & error) {
    require_reconciliation(RegistryError::kInternalError, error.what());
    response->result = to_interface_result(
      {false, RegistryError::kInternalError, before.revision, error.what()});
    return;
  }
  const auto quarantine_resolution =
    request->quarantine_resolution ==
    ReconcileScene::Request::ACCEPT_QUARANTINED_RESOURCES ?
    QuarantineResolution::kAccept : QuarantineResolution::kPreserve;
  const auto reset = registry_->rebase_session(
    new_session_id, quarantine_resolution,
    request->expected_quarantined_resource_keys);
  if (!reset.success) {
    if (reset.error == RegistryError::kResourceQuarantined) {
      expire_leases();
    }
    response->result = to_interface_result(reset);
    return;
  }

  accept_reconciled_scene(readback.scene, readback.digest);
  diagnostics_.setHardwareID(new_session_id);
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    last_reported_expiry_count_ = 0;
  }
  response->new_session_id = new_session_id;
  response->result = to_interface_result(reset);
  RCLCPP_WARN(
    get_logger(),
    "PlanningScene reconciliation accepted authoritative state; new session=%s",
    new_session_id.c_str());
}

RegistryStatus PlanningSceneCoordinatorNode::invalid_request_status(std::string message) const
{
  return {
    false, RegistryError::kInvalidRequest, registry_->snapshot().revision,
    std::move(message)};
}

}  // namespace manipulation::scene
