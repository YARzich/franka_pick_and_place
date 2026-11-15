// Responsibility: Expires leases and stale operation records without violating active ownership.

#include "scene_lease_registry_impl.hpp"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace manipulation::scene
{

std::vector<std::string> SceneLeaseRegistry::Impl::expire_leases()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return purge_expired_locked(read_now_locked());
}

void SceneLeaseRegistry::Impl::erase_lease_locked(const LeaseMap::iterator lease)
{
  for (const auto & key : lease->second.resource_keys) {
    const auto owner = resource_owners_.find(key);
    if (owner != resource_owners_.end() && owner->second == lease->first) {
      resource_owners_.erase(owner);
    }
  }
  leases_.erase(lease);
}

void SceneLeaseRegistry::Impl::expire_lease_locked(const LeaseMap::iterator lease)
{
  for (const auto & key : lease->second.resource_keys) {
    quarantined_resource_keys_.insert(key);
  }
  erase_lease_locked(lease);
  if (expired_lease_count_ != std::numeric_limits<std::uint64_t>::max()) {
    ++expired_lease_count_;
  }
}

std::vector<std::string> SceneLeaseRegistry::Impl::purge_expired_locked(const TimePoint now)
{
  std::vector<std::string> expired;
  for (auto lease = leases_.begin(); lease != leases_.end(); ) {
    if (lease->second.expires_at > now || lease_is_pinned(lease->first)) {
      ++lease;
      continue;
    }

    const auto expired_lease = lease++;
    expired.push_back(expired_lease->first);
    expire_lease_locked(expired_lease);
  }
  return expired;
}

}  // namespace manipulation::scene
