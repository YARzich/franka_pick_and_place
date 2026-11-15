// Responsibility: Provides process-local atomic ownership of logical control resources.

#ifndef MANIPULATION__SKILL__RESOURCE_REGISTRY_HPP_
#define MANIPULATION__SKILL__RESOURCE_REGISTRY_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation/skill/resource_claim.hpp"

namespace manipulation::skill
{

/// Machine-readable outcome of a non-blocking resource acquisition attempt.
enum class ResourceAcquireError : std::uint8_t
{
  kNone,
  kInvalidOwner,
  kEmptyClaims,
  kConflict,
  kGenerationExhausted,
};

/// One active ownership entry returned by ResourceRegistry::snapshot().
struct ResourceOwnershipSnapshot
{
  std::string resource_id;
  std::string owner_id;
  std::uint64_t generation{0U};
};

/// Deterministically ordered diagnostic view captured under the registry lock.
struct ResourceRegistrySnapshot
{
  std::uint64_t last_generation{0U};
  std::vector<ResourceOwnershipSnapshot> active_ownerships;
};

/**
 * Atomically grants exclusive ownership of process-local control resources.
 *
 * This registry performs no ROS communication, waiting, preemption, TTL, or
 * controller switching. All claims in one request are checked and acquired
 * under one mutex, so callers never retain a partial acquisition. A Lease owns
 * shared registry state and can safely outlive the ResourceRegistry wrapper.
 */
class ResourceRegistry final
{
private:
  struct State;

public:
  /// Move-only RAII ownership token for one acquisition generation.
  class Lease final
  {
public:
    Lease(const Lease &) = delete;
    Lease & operator=(const Lease &) = delete;
    Lease(Lease && other) noexcept;
    Lease & operator=(Lease && other) noexcept;
    ~Lease() noexcept;

    /** Return true only while every resource still belongs to this generation. */
    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] std::uint64_t generation() const noexcept;
    [[nodiscard]] const std::string & owner_id() const noexcept;
    [[nodiscard]] const std::vector<std::string> & resource_ids() const noexcept;

private:
    friend class ResourceRegistry;

    Lease(
      std::shared_ptr<State> state, std::uint64_t generation,
      std::string owner_id, std::vector<std::string> resource_ids) noexcept;
    void release() noexcept;

    std::shared_ptr<State> state_;
    std::uint64_t generation_{0U};
    std::string owner_id_;
    std::vector<std::string> resource_ids_;
  };

  /// Result carrying either a complete lease or a machine-readable rejection.
  struct AcquireResult
  {
    ResourceAcquireError error{ResourceAcquireError::kInvalidOwner};
    std::optional<Lease> lease;
    std::string conflicting_resource_id;

    [[nodiscard]] bool acquired() const noexcept;
  };

  ResourceRegistry();
  ~ResourceRegistry();

  ResourceRegistry(const ResourceRegistry &) = delete;
  ResourceRegistry & operator=(const ResourceRegistry &) = delete;
  ResourceRegistry(ResourceRegistry &&) = delete;
  ResourceRegistry & operator=(ResourceRegistry &&) = delete;

  /**
   * Try to acquire every claim without waiting.
   *
   * The owner must be a canonical logical ID and the claim list must be
   * non-empty. Duplicate claims are folded before the atomic conflict check.
   */
  [[nodiscard]] AcquireResult try_acquire(
    std::string owner_id, std::vector<ResourceClaim> claims);

  /// Capture sorted active ownership and the last granted generation.
  [[nodiscard]] ResourceRegistrySnapshot snapshot() const;

private:
  std::shared_ptr<State> state_;
};

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__RESOURCE_REGISTRY_HPP_
