// Responsibility: Validates exclusive logical resource claim identities.

#include "manipulation/skill/resource_claim.hpp"

#include <stdexcept>
#include <utility>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::skill
{

ResourceClaim::ResourceClaim(std::string resource_id)
: resource_id_(std::move(resource_id))
{
  if (!is_canonical_logical_id(resource_id_)) {
    throw std::invalid_argument("Resource ID must be a canonical logical ID");
  }
}

const std::string & ResourceClaim::resource_id() const noexcept
{
  return resource_id_;
}

bool ResourceClaim::operator==(const ResourceClaim & other) const noexcept
{
  return resource_id_ == other.resource_id_;
}

bool ResourceClaim::operator!=(const ResourceClaim & other) const noexcept
{
  return !(*this == other);
}

bool ResourceClaim::operator<(const ResourceClaim & other) const noexcept
{
  return resource_id_ < other.resource_id_;
}

}  // namespace manipulation::skill
