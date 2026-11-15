// Responsibility: Parses canonical lease resource keys into world, attached and ACM identities.

#ifndef MANIPULATION__SCENE__RESOURCE_KEY_HPP_
#define MANIPULATION__SCENE__RESOURCE_KEY_HPP_

#include <optional>
#include <string>
#include <string_view>

namespace manipulation::scene
{

/// Logical PlanningScene resource categories guarded by coordinator leases.
enum class ResourceKind
{
  kWorldObject,
  kAttachedObject,
  kAllowedCollisionPair,
};

/// Parsed representation of one validated canonical resource key.
struct ResourceKey
{
  ResourceKind kind{ResourceKind::kWorldObject};
  std::string canonical;
  std::string first;
  std::string second;
};

/**
 * Parse and validate a canonical PlanningScene resource key.
 *
 * Object keys use `world:<id>` or `attached:<id>`. Allowed-collision-matrix
 * keys use `acm:<first>|<second>`, where `first` is lexicographically smaller
 * than `second`. Components must be non-empty and contain no whitespace,
 * control characters or the reserved `|` separator.
 *
 * This function is thread-safe and owns no state.
 */
std::optional<ResourceKey> parse_resource_key(std::string_view value);

/// Build `world:<id>`, or return `std::nullopt` when `id` is not valid.
std::optional<std::string> make_world_resource_key(std::string_view id);

/// Build `attached:<id>`, or return `std::nullopt` when `id` is not valid.
std::optional<std::string> make_attached_resource_key(std::string_view id);

/**
 * Build the canonical ACM key for two distinct link or object names.
 *
 * The input order is irrelevant; the returned pair is sorted
 * lexicographically. Invalid or equal components return `std::nullopt`.
 */
std::optional<std::string> make_acm_resource_key(
  std::string_view first,
  std::string_view second);

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__RESOURCE_KEY_HPP_
