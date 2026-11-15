// Responsibility: Defines private registry records, indexes and lock-scoped maintenance helpers.

#ifndef SCENE__SCENE_LEASE_REGISTRY_IMPL_HPP_
#define SCENE__SCENE_LEASE_REGISTRY_IMPL_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "manipulation/scene/scene_lease_registry.hpp"

namespace manipulation::scene
{
namespace detail
{

bool is_valid_identifier(std::string_view value);

RegistryStatus make_success(std::uint64_t revision, std::string message);

RegistryStatus make_failure(
  RegistryError error,
  std::uint64_t revision,
  std::string message);

std::optional<std::vector<std::string>> normalize_resource_keys(
  const std::vector<std::string> & keys);

/**
 * Return whether two canonical keys guard scene state that can erase or move
 * the same object implicitly.
 *
 * In addition to exact-key exclusion, `world:<id>` and `attached:<id>` share
 * one object-lifecycle ownership scope. That scope also excludes every ACM
 * pair containing `<id>`, because removing the object implicitly removes its
 * ACM entries. Distinct ACM pairs remain independently leasable.
 */
bool resource_keys_conflict(std::string_view first, std::string_view second);

}  // namespace detail

class SceneLeaseRegistry::Impl
{
public:
  explicit Impl(Options options, NowFunction now);

  AcquireLeaseResult acquire_lease(const AcquireLeaseRequest & request);

  RenewLeaseResult renew_lease(
    const std::string & lease_token,
    Duration requested_ttl);

  ValidateLeaseResult validate_lease(const ValidateLeaseRequest & request);

  RegistryStatus release_lease(const std::string & lease_token);

  BeginOperationResult begin_operation(const BeginOperationRequest & request);

  RegistryStatus complete_operation(
    const std::string & operation_ticket,
    const BackendOperationResult & backend_result);

  RegistryStatus rebase_session(
    const std::string & new_session_id,
    QuarantineResolution quarantine_resolution,
    const std::vector<std::string> & expected_quarantined_resource_keys);

  std::vector<std::string> expire_leases();

  RegistrySnapshot snapshot() const;

private:
  struct Lease
  {
    std::string owner_id;
    std::vector<std::string> resource_keys;
    TimePoint expires_at;
  };

  struct OperationIdentity
  {
    std::string lease_token;
    std::uint64_t expected_revision{0};
    std::vector<std::string> resource_keys;
    std::string request_digest;

    bool operator==(const OperationIdentity & other) const
    {
      return lease_token == other.lease_token &&
             expected_revision == other.expected_revision &&
             resource_keys == other.resource_keys &&
             request_digest == other.request_digest;
    }
  };

  struct OperationRecord
  {
    OperationIdentity identity;
    bool completed{false};
    std::string ticket;
    RegistryStatus result;
  };

  struct ActiveOperation
  {
    std::string operation_id;
    std::string lease_token;
    std::string ticket;
  };

  using LeaseMap = std::unordered_map<std::string, Lease>;

  TimePoint read_now_locked();

  std::uint64_t current_revision() const;

  bool belongs_to_session(std::string_view lease_token) const;

  bool lease_is_pinned(std::string_view lease_token) const;

  static bool compute_expiry(
    TimePoint now,
    Duration ttl,
    TimePoint & expires_at);

  void erase_lease_locked(LeaseMap::iterator lease);

  void expire_lease_locked(LeaseMap::iterator lease);

  std::vector<std::string> purge_expired_locked(TimePoint now);

  Options options_;
  NowFunction now_;
  mutable std::mutex mutex_;
  TimePoint last_now_;
  std::uint64_t revision_{0};
  std::uint64_t next_lease_sequence_{1};
  std::uint64_t next_operation_ticket_sequence_{1};
  std::uint64_t expired_lease_count_{0};
  std::string lease_token_prefix_;
  std::string operation_ticket_prefix_;
  LeaseMap leases_;
  std::unordered_map<std::string, std::string> resource_owners_;
  std::unordered_set<std::string> quarantined_resource_keys_;
  std::unordered_map<std::string, OperationRecord> operations_;
  std::optional<ActiveOperation> active_operation_;
};

}  // namespace manipulation::scene

#endif  // SCENE__SCENE_LEASE_REGISTRY_IMPL_HPP_
