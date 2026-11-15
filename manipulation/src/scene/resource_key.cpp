// Responsibility: Implements strict parsing and canonical formatting of leased scene resource keys.

#include "manipulation/scene/resource_key.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>

namespace manipulation::scene
{
namespace
{

constexpr std::string_view kWorldPrefix{"world:"};
constexpr std::string_view kAttachedPrefix{"attached:"};
constexpr std::string_view kAcmPrefix{"acm:"};

bool is_valid_component(const std::string_view value)
{
  if (value.empty()) {
    return false;
  }

  return std::all_of(value.begin(), value.end(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return character != '|' && std::isspace(byte) == 0 && std::iscntrl(byte) == 0;
  });
}

std::optional<std::string> make_object_key(
  const std::string_view prefix,
  const std::string_view id)
{
  if (!is_valid_component(id)) {
    return std::nullopt;
  }
  return std::string(prefix) + std::string(id);
}

}  // namespace

std::optional<ResourceKey> parse_resource_key(const std::string_view value)
{
  if (value.rfind(kWorldPrefix, 0) == 0) {
    const auto id = value.substr(kWorldPrefix.size());
    if (!is_valid_component(id)) {
      return std::nullopt;
    }
    return ResourceKey{
      ResourceKind::kWorldObject, std::string(value), std::string(id), {}};
  }

  if (value.rfind(kAttachedPrefix, 0) == 0) {
    const auto id = value.substr(kAttachedPrefix.size());
    if (!is_valid_component(id)) {
      return std::nullopt;
    }
    return ResourceKey{
      ResourceKind::kAttachedObject, std::string(value), std::string(id), {}};
  }

  if (value.rfind(kAcmPrefix, 0) != 0) {
    return std::nullopt;
  }

  const auto pair = value.substr(kAcmPrefix.size());
  const auto separator = pair.find('|');
  if (
    separator == std::string_view::npos ||
    pair.find('|', separator + 1) != std::string_view::npos)
  {
    return std::nullopt;
  }

  const auto first = pair.substr(0, separator);
  const auto second = pair.substr(separator + 1);
  if (!is_valid_component(first) || !is_valid_component(second) || first >= second) {
    return std::nullopt;
  }

  return ResourceKey{
    ResourceKind::kAllowedCollisionPair,
    std::string(value),
    std::string(first),
    std::string(second)};
}

std::optional<std::string> make_world_resource_key(const std::string_view id)
{
  return make_object_key(kWorldPrefix, id);
}

std::optional<std::string> make_attached_resource_key(const std::string_view id)
{
  return make_object_key(kAttachedPrefix, id);
}

std::optional<std::string> make_acm_resource_key(
  const std::string_view first,
  const std::string_view second)
{
  if (!is_valid_component(first) || !is_valid_component(second) || first == second) {
    return std::nullopt;
  }

  const auto ordered = first < second ?
    std::pair{first, second} : std::pair{second, first};
  return std::string(kAcmPrefix) + std::string(ordered.first) + "|" +
         std::string(ordered.second);
}

}  // namespace manipulation::scene
