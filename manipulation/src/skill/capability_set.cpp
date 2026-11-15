// Responsibility: Validates, normalizes, and compares immutable capability collections.

#include "manipulation/skill/capability_set.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::skill
{

CapabilitySet::CapabilitySet(std::vector<std::string> capabilities)
: capabilities_(std::move(capabilities))
{
  for (const auto & capability : capabilities_) {
    if (!is_canonical_logical_id(capability)) {
      throw std::invalid_argument(
              "Capability must be a canonical logical ID: '" + capability + "'");
    }
  }
  std::sort(capabilities_.begin(), capabilities_.end());
  capabilities_.erase(
    std::unique(capabilities_.begin(), capabilities_.end()),
    capabilities_.end());
}

CapabilitySet::CapabilitySet(
  const std::initializer_list<std::string> capabilities)
: CapabilitySet(std::vector<std::string>{capabilities}) {}

bool CapabilitySet::empty() const noexcept
{
  return capabilities_.empty();
}

std::size_t CapabilitySet::size() const noexcept
{
  return capabilities_.size();
}

bool CapabilitySet::contains(const std::string_view capability) const noexcept
{
  if (!is_canonical_logical_id(capability)) {
    return false;
  }
  return std::binary_search(
    capabilities_.cbegin(), capabilities_.cend(), capability,
    [] (const auto & left, const auto & right) {return left < right;});
}

bool CapabilitySet::satisfies(const CapabilitySet & required) const noexcept
{
  return std::includes(
    capabilities_.cbegin(), capabilities_.cend(),
    required.capabilities_.cbegin(), required.capabilities_.cend());
}

std::vector<std::string> CapabilitySet::missing(
  const CapabilitySet & required) const
{
  std::vector<std::string> result;
  std::set_difference(
    required.capabilities_.cbegin(), required.capabilities_.cend(),
    capabilities_.cbegin(), capabilities_.cend(),
    std::back_inserter(result));
  return result;
}

const std::vector<std::string> & CapabilitySet::values() const noexcept
{
  return capabilities_;
}

bool CapabilitySet::operator==(const CapabilitySet & other) const noexcept
{
  return capabilities_ == other.capabilities_;
}

bool CapabilitySet::operator!=(const CapabilitySet & other) const noexcept
{
  return !(*this == other);
}

}  // namespace manipulation::skill
