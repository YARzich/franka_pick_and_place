// Responsibility: Implements atomic acquire, renew, release and ownership checks for resource
//   leases.

#include "scene_lease_registry_impl.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace manipulation::scene
{

AcquireLeaseResult SceneLeaseRegistry::Impl::acquire_lease(
  const AcquireLeaseRequest & request)
{
  const auto normalized_keys = detail::normalize_resource_keys(request.resource_keys);
  if (!detail::is_valid_identifier(request.owner_id) || !normalized_keys ||
    request.requested_ttl <= Duration::zero())
  {
    return AcquireLeaseResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, current_revision(),
        "owner, canonical unique resources and a positive TTL are required"),
      {}, Duration::zero(), {}};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  purge_expired_locked(now);

  TimePoint expires_at;
  if (!compute_expiry(now, request.requested_ttl, expires_at)) {
    return AcquireLeaseResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, revision_, "requested TTL exceeds clock range"),
      {}, Duration::zero(), {}};
  }

  for (const auto & requested_key : *normalized_keys) {
    std::string quarantined_conflict;
    for (const auto & existing_key : quarantined_resource_keys_) {
      if (
        detail::resource_keys_conflict(requested_key, existing_key) &&
        (quarantined_conflict.empty() || existing_key < quarantined_conflict))
      {
        quarantined_conflict = existing_key;
      }
    }
    if (!quarantined_conflict.empty()) {
      return AcquireLeaseResult{
        detail::make_failure(
          RegistryError::kResourceQuarantined, revision_,
          "a requested resource overlaps quarantined scene ownership"),
        {}, Duration::zero(), quarantined_conflict};
    }

    std::string owned_conflict;
    for (const auto & existing : resource_owners_) {
      if (
        detail::resource_keys_conflict(requested_key, existing.first) &&
        (owned_conflict.empty() || existing.first < owned_conflict))
      {
        owned_conflict = existing.first;
      }
    }
    if (!owned_conflict.empty()) {
      return AcquireLeaseResult{
        detail::make_failure(
          RegistryError::kResourceConflict, revision_,
          "a requested resource overlaps an existing scene lease"),
        {}, Duration::zero(), owned_conflict};
    }
  }

  const auto lease_token = lease_token_prefix_ + std::to_string(next_lease_sequence_++);
  leases_.emplace(
    lease_token,
    Lease{request.owner_id, *normalized_keys, expires_at});
  for (const auto & key : *normalized_keys) {
    resource_owners_.emplace(key, lease_token);
  }

  return AcquireLeaseResult{
    detail::make_success(revision_, "lease acquired"),
    lease_token,
    request.requested_ttl,
    {}};
}

RenewLeaseResult SceneLeaseRegistry::Impl::renew_lease(
  const std::string & lease_token,
  const Duration requested_ttl)
{
  if (!detail::is_valid_identifier(lease_token) || requested_ttl <= Duration::zero()) {
    return RenewLeaseResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, current_revision(),
        "lease token and a positive TTL are required"),
      Duration::zero()};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  if (!belongs_to_session(lease_token)) {
    purge_expired_locked(now);
    return RenewLeaseResult{
      detail::make_failure(
        RegistryError::kSessionMismatch, revision_,
        "lease token belongs to another coordinator session"),
      Duration::zero()};
  }

  auto lease = leases_.find(lease_token);
  if (lease == leases_.end()) {
    purge_expired_locked(now);
    return RenewLeaseResult{
      detail::make_failure(
        RegistryError::kLeaseNotFound, revision_, "lease token was not found"),
      Duration::zero()};
  }
  if (lease->second.expires_at <= now) {
    if (!lease_is_pinned(lease_token)) {
      expire_lease_locked(lease);
    }
    purge_expired_locked(now);
    return RenewLeaseResult{
      detail::make_failure(
        RegistryError::kLeaseExpired, revision_, "an expired lease cannot be renewed"),
      Duration::zero()};
  }

  TimePoint expires_at;
  if (!compute_expiry(now, requested_ttl, expires_at)) {
    return RenewLeaseResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, revision_, "requested TTL exceeds clock range"),
      Duration::zero()};
  }

  lease->second.expires_at = expires_at;
  purge_expired_locked(now);
  return RenewLeaseResult{
    detail::make_success(revision_, "lease renewed"), requested_ttl};
}

ValidateLeaseResult SceneLeaseRegistry::Impl::validate_lease(
  const ValidateLeaseRequest & request)
{
  const auto normalized_keys = detail::normalize_resource_keys(
    request.required_resource_keys);
  if (!detail::is_valid_identifier(request.lease_token) || !normalized_keys) {
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kInvalidRequest, current_revision(),
        "lease token and canonical unique required resources are required"),
      options_.session_id, Duration::zero()};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  if (!belongs_to_session(request.lease_token)) {
    purge_expired_locked(now);
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kSessionMismatch, revision_,
        "lease token belongs to another coordinator session"),
      options_.session_id, Duration::zero()};
  }

  auto lease = leases_.find(request.lease_token);
  if (lease == leases_.end()) {
    purge_expired_locked(now);
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kLeaseNotFound, revision_, "active lease was not found"),
      options_.session_id, Duration::zero()};
  }
  if (lease->second.expires_at <= now) {
    if (!lease_is_pinned(request.lease_token)) {
      expire_lease_locked(lease);
    }
    purge_expired_locked(now);
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kLeaseExpired, revision_, "lease lifetime has elapsed"),
      options_.session_id, Duration::zero()};
  }

  purge_expired_locked(now);
  if (active_operation_) {
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kOperationInProgress, revision_,
        "a PlanningScene mutation is currently in progress"),
      options_.session_id, Duration::zero()};
  }
  if (request.expected_revision != revision_) {
    return ValidateLeaseResult{
      detail::make_failure(
        RegistryError::kRevisionMismatch, revision_,
        "expected revision does not match the current scene revision"),
      options_.session_id, Duration::zero()};
  }
  for (const auto & key : *normalized_keys) {
    if (!std::binary_search(
        lease->second.resource_keys.begin(), lease->second.resource_keys.end(), key))
    {
      return ValidateLeaseResult{
        detail::make_failure(
          RegistryError::kNotOwner, revision_,
          "lease does not own every required resource"),
        options_.session_id, Duration::zero()};
    }
  }

  return ValidateLeaseResult{
    detail::make_success(revision_, "lease is active and owns required resources"),
    options_.session_id,
    lease->second.expires_at - now};
}

RegistryStatus SceneLeaseRegistry::Impl::release_lease(const std::string & lease_token)
{
  if (!detail::is_valid_identifier(lease_token)) {
    return detail::make_failure(
      RegistryError::kInvalidRequest, current_revision(), "lease token is required");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = read_now_locked();
  if (!belongs_to_session(lease_token)) {
    purge_expired_locked(now);
    return detail::make_failure(
      RegistryError::kSessionMismatch, revision_,
      "lease token belongs to another coordinator session");
  }

  auto lease = leases_.find(lease_token);
  if (lease == leases_.end()) {
    purge_expired_locked(now);
    return detail::make_failure(
      RegistryError::kLeaseNotFound, revision_, "lease token was not found");
  }
  if (lease_is_pinned(lease_token)) {
    return detail::make_failure(
      RegistryError::kOperationInProgress, revision_,
      "lease is pinned by an in-flight backend operation");
  }
  if (lease->second.expires_at <= now) {
    expire_lease_locked(lease);
    purge_expired_locked(now);
    return detail::make_failure(
      RegistryError::kLeaseExpired, revision_, "lease had already expired");
  }

  erase_lease_locked(lease);
  purge_expired_locked(now);
  return detail::make_success(revision_, "lease released");
}

}  // namespace manipulation::scene
