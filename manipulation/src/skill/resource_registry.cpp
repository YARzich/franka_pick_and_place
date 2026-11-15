// Responsibility: Implements atomic generation-safe process-local resource ownership.

#include "manipulation/skill/resource_registry.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::skill
{

struct ResourceRegistry::State final
{
  struct Ownership final
  {
    std::string owner_id;
    std::uint64_t generation{0U};
  };

  mutable std::mutex mutex;
  std::uint64_t last_generation{0U};
  std::unordered_map<std::string, Ownership> ownerships;
};

ResourceRegistry::Lease::Lease(
  std::shared_ptr<State> state, const std::uint64_t generation,
  std::string owner_id, std::vector<std::string> resource_ids) noexcept
: state_(std::move(state)),
  generation_(generation),
  owner_id_(std::move(owner_id)),
  resource_ids_(std::move(resource_ids)) {}

ResourceRegistry::Lease::Lease(Lease && other) noexcept
: state_(std::move(other.state_)),
  generation_(std::exchange(other.generation_, 0U)),
  owner_id_(std::move(other.owner_id_)),
  resource_ids_(std::move(other.resource_ids_)) {}

ResourceRegistry::Lease & ResourceRegistry::Lease::operator=(Lease && other) noexcept
{
  if (this != &other) {
    release();
    state_ = std::move(other.state_);
    generation_ = std::exchange(other.generation_, 0U);
    owner_id_ = std::move(other.owner_id_);
    resource_ids_ = std::move(other.resource_ids_);
  }
  return *this;
}

ResourceRegistry::Lease::~Lease() noexcept
{
  release();
}

bool ResourceRegistry::Lease::valid() const noexcept
{
  if (!state_ || generation_ == 0U || resource_ids_.empty()) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(state_->mutex);
  return std::all_of(
    resource_ids_.cbegin(), resource_ids_.cend(),
    [this](const auto & resource_id) {
      const auto ownership = state_->ownerships.find(resource_id);
      return ownership != state_->ownerships.cend() &&
             ownership->second.generation == generation_ &&
             ownership->second.owner_id == owner_id_;
    });
}

std::uint64_t ResourceRegistry::Lease::generation() const noexcept
{
  return generation_;
}

const std::string & ResourceRegistry::Lease::owner_id() const noexcept
{
  return owner_id_;
}

const std::vector<std::string> & ResourceRegistry::Lease::resource_ids() const noexcept
{
  return resource_ids_;
}

void ResourceRegistry::Lease::release() noexcept
{
  if (!state_ || generation_ == 0U) {
    return;
  }

  // Keep State alive until after the lock is released when this lease outlives
  // and becomes the last owner after the ResourceRegistry wrapper is destroyed.
  const auto state = state_;
  {
    const std::lock_guard<std::mutex> lock(state->mutex);
    for (const auto & resource_id : resource_ids_) {
      const auto ownership = state->ownerships.find(resource_id);
      if (ownership != state->ownerships.end() &&
        ownership->second.generation == generation_ &&
        ownership->second.owner_id == owner_id_)
      {
        state->ownerships.erase(ownership);
      }
    }
  }
  state_.reset();
  generation_ = 0U;
  owner_id_.clear();
  resource_ids_.clear();
}

bool ResourceRegistry::AcquireResult::acquired() const noexcept
{
  return error == ResourceAcquireError::kNone && lease.has_value() && lease->valid();
}

ResourceRegistry::ResourceRegistry()
: state_(std::make_shared<State>()) {}

ResourceRegistry::~ResourceRegistry() = default;

ResourceRegistry::AcquireResult ResourceRegistry::try_acquire(
  std::string owner_id, std::vector<ResourceClaim> claims)
{
  if (!is_canonical_logical_id(owner_id)) {
    return {ResourceAcquireError::kInvalidOwner, std::nullopt, {}};
  }
  if (claims.empty()) {
    return {ResourceAcquireError::kEmptyClaims, std::nullopt, {}};
  }

  std::sort(claims.begin(), claims.end());
  claims.erase(std::unique(claims.begin(), claims.end()), claims.end());
  std::vector<std::string> resource_ids;
  resource_ids.reserve(claims.size());
  for (const auto & claim : claims) {
    resource_ids.push_back(claim.resource_id());
  }

  const std::lock_guard<std::mutex> lock(state_->mutex);
  for (const auto & resource_id : resource_ids) {
    if (state_->ownerships.count(resource_id) != 0U) {
      return {ResourceAcquireError::kConflict, std::nullopt, resource_id};
    }
  }
  if (state_->last_generation == std::numeric_limits<std::uint64_t>::max()) {
    return {ResourceAcquireError::kGenerationExhausted, std::nullopt, {}};
  }

  const std::uint64_t generation = state_->last_generation + 1U;
  auto updated_ownerships = state_->ownerships;
  for (const auto & resource_id : resource_ids) {
    updated_ownerships.emplace(
      resource_id, State::Ownership{owner_id, generation});
  }
  AcquireResult result;
  result.error = ResourceAcquireError::kNone;
  result.lease.emplace(
    Lease{state_, generation, std::move(owner_id), std::move(resource_ids)});

  // Publish both pieces of registry state only after every allocation and copy
  // above has succeeded. This preserves the strong exception guarantee.
  state_->ownerships.swap(updated_ownerships);
  state_->last_generation = generation;
  return result;
}

ResourceRegistrySnapshot ResourceRegistry::snapshot() const
{
  ResourceRegistrySnapshot result;
  const std::lock_guard<std::mutex> lock(state_->mutex);
  result.last_generation = state_->last_generation;
  result.active_ownerships.reserve(state_->ownerships.size());
  for (const auto & [resource_id, ownership] : state_->ownerships) {
    result.active_ownerships.push_back(
      ResourceOwnershipSnapshot{
        resource_id, ownership.owner_id, ownership.generation});
  }
  std::sort(
    result.active_ownerships.begin(), result.active_ownerships.end(),
    [](const auto & left, const auto & right) {
      return left.resource_id < right.resource_id;
    });
  return result;
}

}  // namespace manipulation::skill
