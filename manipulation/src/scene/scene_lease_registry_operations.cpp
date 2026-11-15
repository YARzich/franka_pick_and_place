// Responsibility: Implements idempotent operation admission, replay lookup and terminal completion
//   records.

#include "scene_lease_registry_impl.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace manipulation::scene
{

BeginOperationResult SceneLeaseRegistry::Impl::begin_operation(
  const BeginOperationRequest & request)
{
  const auto normalized_keys = detail::normalize_resource_keys(request.resource_keys);
  if (!detail::is_valid_identifier(request.lease_token) ||
    !detail::is_valid_identifier(request.operation_id) ||
    !detail::is_valid_identifier(request.request_digest) || !normalized_keys)
  {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, current_revision(),
        "lease, operation ID, digest and canonical unique resources are required"),
      OperationDisposition::kRejected,
      {}};
  }

  const OperationIdentity identity{
    request.lease_token,
    request.expected_revision,
    *normalized_keys,
    request.request_digest};

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  purge_expired_locked(now);

  const auto recorded = operations_.find(request.operation_id);
  if (recorded != operations_.end()) {
    if (!(recorded->second.identity == identity)) {
      return BeginOperationResult{
        detail::make_failure(
          RegistryError::kIdempotencyConflict, revision_,
          "operation ID was reused with different request content"),
        OperationDisposition::kRejected,
        {}};
    }
    if (!recorded->second.completed) {
      return BeginOperationResult{
        detail::make_failure(
          RegistryError::kOperationInProgress, revision_,
          "the same operation is still in progress"),
        OperationDisposition::kRejected,
        {}};
    }
    return BeginOperationResult{
      recorded->second.result, OperationDisposition::kReplay, {}};
  }

  if (operations_.size() >= options_.max_operation_records) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kOperationCapacityExceeded, revision_,
        "operation record capacity is exhausted for this session"),
      OperationDisposition::kRejected,
      {}};
  }
  if (revision_ == std::numeric_limits<std::uint64_t>::max()) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kInternalError, revision_,
        "scene revision cannot represent another successful mutation"),
      OperationDisposition::kRejected,
      {}};
  }
  if (!belongs_to_session(request.lease_token)) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kSessionMismatch, revision_,
        "lease token belongs to another coordinator session"),
      OperationDisposition::kRejected,
      {}};
  }

  const auto lease = leases_.find(request.lease_token);
  if (lease == leases_.end()) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kLeaseNotFound, revision_, "active lease was not found"),
      OperationDisposition::kRejected,
      {}};
  }
  if (active_operation_) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kOperationInProgress, revision_,
        "another scene operation is in progress"),
      OperationDisposition::kRejected,
      {}};
  }
  if (request.expected_revision != revision_) {
    return BeginOperationResult{
      detail::make_failure(
        RegistryError::kRevisionMismatch, revision_,
        "expected revision does not match the current scene revision"),
      OperationDisposition::kRejected,
      {}};
  }
  for (const auto & key : *normalized_keys) {
    const auto owner = resource_owners_.find(key);
    if (owner == resource_owners_.end() || owner->second != request.lease_token) {
      return BeginOperationResult{
        detail::make_failure(
          RegistryError::kNotOwner, revision_,
          "operation references a resource not owned by its lease"),
        OperationDisposition::kRejected,
        {}};
    }
  }

  const auto ticket = operation_ticket_prefix_ +
    std::to_string(next_operation_ticket_sequence_++);
  operations_.emplace(
    request.operation_id,
    OperationRecord{identity, false, ticket, {}});
  active_operation_ = ActiveOperation{request.operation_id, request.lease_token, ticket};

  return BeginOperationResult{
    detail::make_success(revision_, "operation authorized"),
    OperationDisposition::kReady,
    ticket};
}

RegistryStatus SceneLeaseRegistry::Impl::complete_operation(
  const std::string & operation_ticket,
  const BackendOperationResult & backend_result)
{
  if (!detail::is_valid_identifier(operation_ticket)) {
    return detail::make_failure(
      RegistryError::kInvalidRequest, current_revision(), "operation ticket is required");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  if (!active_operation_ || active_operation_->ticket != operation_ticket) {
    purge_expired_locked(now);
    return detail::make_failure(
      RegistryError::kOperationTicketInvalid, revision_,
      "operation ticket is unknown or already completed");
  }

  const auto operation = operations_.find(active_operation_->operation_id);
  if (operation == operations_.end() || operation->second.completed ||
    operation->second.ticket != operation_ticket)
  {
    active_operation_.reset();
    purge_expired_locked(now);
    return detail::make_failure(
      RegistryError::kInternalError, revision_,
      "registry operation state is inconsistent");
  }

  RegistryStatus result;
  if (backend_result.success) {
    ++revision_;
    result = detail::make_success(
      revision_, backend_result.message.empty() ?
      "backend operation completed" : backend_result.message);
  } else {
    const auto failure_error =
      backend_result.failure_error == RegistryError::kBackendFailed ||
      backend_result.failure_error == RegistryError::kTimeout ||
      backend_result.failure_error == RegistryError::kInternalError ?
      backend_result.failure_error : RegistryError::kInternalError;
    result = detail::make_failure(
      failure_error,
      revision_,
      backend_result.message.empty() ?
      "PlanningScene backend rejected the operation" : backend_result.message);
  }

  operation->second.completed = true;
  operation->second.result = result;
  active_operation_.reset();
  purge_expired_locked(now);
  return result;
}

RegistryStatus SceneLeaseRegistry::Impl::rebase_session(
  const std::string & new_session_id,
  const QuarantineResolution quarantine_resolution,
  const std::vector<std::string> & expected_quarantined_resource_keys)
{
  if (!detail::is_valid_identifier(new_session_id)) {
    return detail::make_failure(
      RegistryError::kInvalidRequest, current_revision(),
      "new session ID must be a non-empty token without whitespace");
  }
  if (quarantine_resolution != QuarantineResolution::kPreserve &&
    quarantine_resolution != QuarantineResolution::kAccept)
  {
    return detail::make_failure(
      RegistryError::kInvalidRequest, current_revision(),
      "quarantine resolution uses an unknown value");
  }
  if (quarantine_resolution == QuarantineResolution::kPreserve &&
    !expected_quarantined_resource_keys.empty())
  {
    return detail::make_failure(
      RegistryError::kInvalidRequest, current_revision(),
      "preserve quarantine requests must not acknowledge resource keys");
  }
  std::vector<std::string> expected_keys;
  if (!expected_quarantined_resource_keys.empty()) {
    const auto normalized = detail::normalize_resource_keys(
      expected_quarantined_resource_keys);
    if (!normalized) {
      return detail::make_failure(
        RegistryError::kInvalidRequest, current_revision(),
        "expected quarantine keys must be canonical and unique");
    }
    expected_keys = *normalized;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  purge_expired_locked(read_now_locked());
  if (active_operation_) {
    return detail::make_failure(
      RegistryError::kOperationInProgress, revision_,
      "cannot rebase while a backend operation is active");
  }
  if (new_session_id == options_.session_id) {
    return detail::make_failure(
      RegistryError::kInvalidRequest, revision_,
      "reconciliation must create a distinct coordinator session");
  }
  if (!quarantined_resource_keys_.empty() &&
    quarantine_resolution != QuarantineResolution::kAccept)
  {
    return detail::make_failure(
      RegistryError::kResourceQuarantined, revision_,
      "session rebase requires explicit quarantine acceptance");
  }
  std::vector<std::string> actual_keys(
    quarantined_resource_keys_.begin(), quarantined_resource_keys_.end());
  std::sort(actual_keys.begin(), actual_keys.end());
  if (quarantine_resolution == QuarantineResolution::kAccept &&
    expected_keys != actual_keys)
  {
    return detail::make_failure(
      RegistryError::kResourceQuarantined, revision_,
      "quarantine changed before recovery acknowledgement");
  }

  options_.session_id = new_session_id;
  lease_token_prefix_ = options_.session_id + "/lease/";
  operation_ticket_prefix_ = options_.session_id + "/operation/";
  leases_.clear();
  resource_owners_.clear();
  quarantined_resource_keys_.clear();
  operations_.clear();
  next_lease_sequence_ = 1;
  next_operation_ticket_sequence_ = 1;
  expired_lease_count_ = 0;
  return detail::make_success(revision_, "coordinator session rebased after reconciliation");
}

}  // namespace manipulation::scene
