// Responsibility: Validates and canonicalizes immutable skill execution metadata.

#include "manipulation/skill/skill_descriptor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::skill
{

SkillDescriptor SkillDescriptor::create(
  std::string skill_type,
  PresetIdentity preset,
  CapabilitySet required_capabilities,
  std::vector<ResourceClaim> resource_claims,
  std::optional<PresetIdentity> deployment_identity)
{
  if (!is_canonical_logical_id(skill_type)) {
    throw std::invalid_argument("Skill type must be a canonical logical ID");
  }
  std::sort(resource_claims.begin(), resource_claims.end());
  resource_claims.erase(
    std::unique(resource_claims.begin(), resource_claims.end()),
    resource_claims.end());
  return SkillDescriptor(
    std::move(skill_type), std::move(preset),
    std::move(required_capabilities), std::move(resource_claims),
    std::move(deployment_identity));
}

const std::string & SkillDescriptor::skill_type() const noexcept
{
  return skill_type_;
}

const PresetIdentity & SkillDescriptor::preset() const noexcept
{
  return preset_;
}

const std::optional<PresetIdentity> & SkillDescriptor::deployment_identity() const noexcept
{
  return deployment_identity_;
}

const CapabilitySet & SkillDescriptor::required_capabilities() const noexcept
{
  return required_capabilities_;
}

const std::vector<ResourceClaim> & SkillDescriptor::resource_claims() const noexcept
{
  return resource_claims_;
}

SkillDescriptor::SkillDescriptor(
  std::string skill_type,
  PresetIdentity preset,
  CapabilitySet required_capabilities,
  std::vector<ResourceClaim> resource_claims,
  std::optional<PresetIdentity> deployment_identity)
: skill_type_(std::move(skill_type)),
  preset_(std::move(preset)),
  deployment_identity_(std::move(deployment_identity)),
  required_capabilities_(std::move(required_capabilities)),
  resource_claims_(std::move(resource_claims))
{
}

}  // namespace manipulation::skill
