// Responsibility: Handles validated leased scene mutations with revision and replay protection.

#include "planning_scene_coordinator_node.hpp"

#include <exception>
#include <optional>
#include <string>

#include "operation_completion_guard.hpp"

namespace manipulation::scene
{

void PlanningSceneCoordinatorNode::handle_acquire(
  const AcquireLease::Request::SharedPtr request,
  AcquireLease::Response::SharedPtr response)
{
  const auto ttl = parse_positive_steady_duration(request->requested_ttl);
  const auto granted_ttl = ttl.valid ? to_duration_message(ttl.value) : std::nullopt;
  if (!ttl.valid || !granted_ttl) {
    response->result = to_interface_result(
      invalid_request_status(ttl.valid ? "TTL exceeds interface range" : ttl.message));
    return;
  }

  const auto result = registry_->acquire_lease(
    {request->owner_id, request->resource_keys, ttl.value});
  response->result = to_interface_result(result.status);
  if (!result.status.success) {
    return;
  }

  response->lease_id = result.lease_token;
  response->granted_ttl = *granted_ttl;
}

void PlanningSceneCoordinatorNode::handle_renew(
  const RenewLease::Request::SharedPtr request,
  RenewLease::Response::SharedPtr response)
{
  const auto ttl = parse_positive_steady_duration(request->requested_ttl);
  const auto granted_ttl = ttl.valid ? to_duration_message(ttl.value) : std::nullopt;
  if (!ttl.valid || !granted_ttl) {
    response->result = to_interface_result(
      invalid_request_status(ttl.valid ? "TTL exceeds interface range" : ttl.message));
    return;
  }

  const auto result = registry_->renew_lease(request->lease_id, ttl.value);
  response->result = to_interface_result(result.status);
  if (result.status.success) {
    response->granted_ttl = *granted_ttl;
  }
}

void PlanningSceneCoordinatorNode::handle_release(
  const ReleaseLease::Request::SharedPtr request,
  ReleaseLease::Response::SharedPtr response)
{
  response->result = to_interface_result(registry_->release_lease(request->lease_id));
}

void PlanningSceneCoordinatorNode::handle_validate(
  const ValidateLease::Request::SharedPtr request,
  ValidateLease::Response::SharedPtr response)
{
  auto result = registry_->validate_lease(
    {request->lease_id, request->required_resource_keys, request->expected_revision});
  response->session_id = result.session_id;
  response->result = to_interface_result(result.status);
  if (!result.status.success) {
    return;
  }

  const auto consistency = ensure_reconciled();
  if (!consistency.success) {
    response->result = to_interface_result(consistency);
    return;
  }

  // Readback can consume a meaningful part of a short TTL. Revalidate after
  // it so remaining_ttl and revision describe the actual return boundary.
  result = registry_->validate_lease(
    {request->lease_id, request->required_resource_keys, request->expected_revision});
  response->session_id = result.session_id;
  response->result = to_interface_result(result.status);
  if (!result.status.success) {
    return;
  }

  const auto remaining_ttl = to_duration_message(result.remaining_ttl);
  if (!remaining_ttl) {
    response->result = to_interface_result(
      {false, RegistryError::kInternalError, result.status.revision,
        "remaining lease TTL exceeds interface range"});
    return;
  }
  response->remaining_ttl = *remaining_ttl;
}

void PlanningSceneCoordinatorNode::handle_apply(
  const ApplySceneDiff::Request::SharedPtr request,
  ApplySceneDiff::Response::SharedPtr response)
{
  const auto validation = validate_scene_diff_resources(
    request->scene_diff, request->resource_keys);
  if (!validation.valid) {
    response->result = to_interface_result(invalid_request_status(validation.message));
    return;
  }

  const auto consistency = ensure_reconciled();
  if (!consistency.success) {
    response->result = to_interface_result(consistency);
    return;
  }

  const auto dependencies = validate_scene_dependencies(request->scene_diff);
  if (!dependencies.success) {
    response->result = to_interface_result(dependencies);
    return;
  }

  std::string digest;
  try {
    digest = digest_apply_request(*request);
  } catch (const std::exception & error) {
    response->result = to_interface_result(
      {false, RegistryError::kInternalError, registry_->snapshot().revision, error.what()});
    return;
  }

  const auto begin = registry_->begin_operation(
    {request->lease_id, request->operation_id, request->expected_revision,
      validation.canonical_resource_keys, digest});
  if (begin.disposition != OperationDisposition::kReady) {
    response->result = to_interface_result(begin.status);
    return;
  }

  OperationCompletionGuard completion(*registry_, begin.operation_ticket);
  BackendApplyAttempt attempt;
  try {
    attempt = apply_to_moveit(request->scene_diff, begin.operation_ticket);
  } catch (const std::exception & error) {
    attempt = {
      true,
      make_backend_operation_result(BackendCallOutcome::kInternalError, error.what()),
      false};
    require_reconciliation(RegistryError::kInternalError, error.what());
  }
  if (!attempt.terminal) {
    completion.release_to_async_completion();
    const auto snapshot = registry_->snapshot();
    response->result = to_interface_result(
      {false, RegistryError::kTimeout, snapshot.revision,
        "backend outcome is ambiguous; coordinator is fail-closed until late "
        "response and readback"});
    return;
  }

  const auto status = completion.complete(attempt.result);
  expire_leases();
  record_backend_status(status);
  if (attempt.reconciliation_confirmed) {
    response->result = to_interface_result(status);
  } else {
    response->result = to_interface_result(
      {false, RegistryError::kReconciliationRequired, status.revision,
        "backend response could not be reconciled with authoritative PlanningScene"});
  }
}

}  // namespace manipulation::scene
