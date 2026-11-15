// Responsibility: Initializes and validates registry limits shared by lease and operation tracking.

#include "manipulation/scene/scene_lease_registry.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "manipulation/scene/resource_key.hpp"
#include "scene_lease_registry_impl.hpp"

namespace manipulation::scene
{
namespace detail
{

bool is_valid_identifier(const std::string_view value)
{
  if (value.empty()) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return std::isspace(byte) == 0 && std::iscntrl(byte) == 0;
  });
}

RegistryStatus make_success(const std::uint64_t revision, std::string message)
{
  return RegistryStatus{true, RegistryError::kNone, revision, std::move(message)};
}

RegistryStatus make_failure(
  const RegistryError error,
  const std::uint64_t revision,
  std::string message)
{
  return RegistryStatus{false, error, revision, std::move(message)};
}

std::optional<std::vector<std::string>> normalize_resource_keys(
  const std::vector<std::string> & keys)
{
  if (keys.empty()) {
    return std::nullopt;
  }

  std::vector<std::string> normalized;
  normalized.reserve(keys.size());
  for (const auto & key : keys) {
    const auto parsed = parse_resource_key(key);
    if (!parsed) {
      return std::nullopt;
    }
    normalized.push_back(parsed->canonical);
  }

  std::sort(normalized.begin(), normalized.end());
  if (std::adjacent_find(normalized.begin(), normalized.end()) != normalized.end()) {
    return std::nullopt;
  }
  return normalized;
}

bool resource_keys_conflict(
  const std::string_view first,
  const std::string_view second)
{
  if (first == second) {
    return true;
  }

  const auto first_key = parse_resource_key(first);
  const auto second_key = parse_resource_key(second);
  if (!first_key || !second_key) {
    return false;
  }

  const auto is_object_key = [](const ResourceKind kind) {
      return kind == ResourceKind::kWorldObject ||
             kind == ResourceKind::kAttachedObject;
    };
  const auto acm_contains = [](const ResourceKey & acm, const std::string & object_id) {
      return acm.first == object_id || acm.second == object_id;
    };

  if (is_object_key(first_key->kind) && is_object_key(second_key->kind)) {
    return first_key->first == second_key->first;
  }
  if (
    is_object_key(first_key->kind) &&
    second_key->kind == ResourceKind::kAllowedCollisionPair)
  {
    return acm_contains(*second_key, first_key->first);
  }
  if (
    first_key->kind == ResourceKind::kAllowedCollisionPair &&
    is_object_key(second_key->kind))
  {
    return acm_contains(*first_key, second_key->first);
  }
  return false;
}

}  // namespace detail

SceneLeaseRegistry::Impl::Impl(Options options, NowFunction now)
: options_(std::move(options)), now_(std::move(now)), revision_(options_.initial_revision)
{
  if (!detail::is_valid_identifier(options_.session_id)) {
    throw std::invalid_argument("session_id must be a non-empty token without whitespace");
  }
  if (options_.max_operation_records == 0) {
    throw std::invalid_argument("max_operation_records must be positive");
  }
  if (!now_) {
    throw std::invalid_argument("now callback must be callable");
  }
  last_now_ = now_();
  lease_token_prefix_ = options_.session_id + "/lease/";
  operation_ticket_prefix_ = options_.session_id + "/operation/";
}

SceneLeaseRegistry::TimePoint SceneLeaseRegistry::Impl::read_now_locked()
{
  const auto observed = now_();
  if (observed > last_now_) {
    last_now_ = observed;
  }
  return last_now_;
}

std::uint64_t SceneLeaseRegistry::Impl::current_revision() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return revision_;
}

bool SceneLeaseRegistry::Impl::belongs_to_session(const std::string_view lease_token) const
{
  return lease_token.rfind(lease_token_prefix_, 0) == 0;
}

bool SceneLeaseRegistry::Impl::lease_is_pinned(const std::string_view lease_token) const
{
  return active_operation_ && active_operation_->lease_token == lease_token;
}

bool SceneLeaseRegistry::Impl::compute_expiry(
  const TimePoint now,
  const Duration ttl,
  TimePoint & expires_at)
{
  if (ttl <= Duration::zero() || ttl > TimePoint::max() - now) {
    return false;
  }
  expires_at = now + ttl;
  return true;
}

RegistrySnapshot SceneLeaseRegistry::Impl::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> quarantined_resource_keys(
    quarantined_resource_keys_.begin(), quarantined_resource_keys_.end());
  std::sort(quarantined_resource_keys.begin(), quarantined_resource_keys.end());
  return RegistrySnapshot{
    options_.session_id,
    revision_,
    leases_.size(),
    operations_.size(),
    options_.max_operation_records,
    operations_.size() >= options_.max_operation_records,
    active_operation_.has_value(),
    expired_lease_count_,
    std::move(quarantined_resource_keys)};
}

SceneLeaseRegistry::SceneLeaseRegistry(Options options, NowFunction now)
: impl_(std::make_unique<Impl>(std::move(options), std::move(now)))
{
}

SceneLeaseRegistry::~SceneLeaseRegistry() = default;

AcquireLeaseResult SceneLeaseRegistry::acquire_lease(const AcquireLeaseRequest & request)
{
  return impl_->acquire_lease(request);
}

RenewLeaseResult SceneLeaseRegistry::renew_lease(
  const std::string & lease_token,
  const Duration requested_ttl)
{
  return impl_->renew_lease(lease_token, requested_ttl);
}

ValidateLeaseResult SceneLeaseRegistry::validate_lease(
  const ValidateLeaseRequest & request)
{
  return impl_->validate_lease(request);
}

RegistryStatus SceneLeaseRegistry::release_lease(const std::string & lease_token)
{
  return impl_->release_lease(lease_token);
}

BeginOperationResult SceneLeaseRegistry::begin_operation(const BeginOperationRequest & request)
{
  return impl_->begin_operation(request);
}

RegistryStatus SceneLeaseRegistry::complete_operation(
  const std::string & operation_ticket,
  const BackendOperationResult & backend_result)
{
  return impl_->complete_operation(operation_ticket, backend_result);
}

RegistryStatus SceneLeaseRegistry::rebase_session(
  const std::string & new_session_id,
  const QuarantineResolution quarantine_resolution,
  const std::vector<std::string> & expected_quarantined_resource_keys)
{
  return impl_->rebase_session(
    new_session_id, quarantine_resolution, expected_quarantined_resource_keys);
}

std::vector<std::string> SceneLeaseRegistry::expire_leases()
{
  return impl_->expire_leases();
}

RegistrySnapshot SceneLeaseRegistry::snapshot() const
{
  return impl_->snapshot();
}

}  // namespace manipulation::scene
