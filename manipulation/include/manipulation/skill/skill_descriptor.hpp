// Responsibility: Groups one skill type, immutable preset, capabilities, and resource claims.

#ifndef MANIPULATION__SKILL__SKILL_DESCRIPTOR_HPP_
#define MANIPULATION__SKILL__SKILL_DESCRIPTOR_HPP_

#include <optional>
#include <string>
#include <vector>

#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/preset_identity.hpp"
#include "manipulation/skill/resource_claim.hpp"

namespace manipulation::skill
{

/**
 * Immutable metadata needed to admit, arbitrate, and record one skill type.
 *
 * Capabilities describe what the deployment must provide. Resource claims
 * describe which physical command paths must be exclusively owned while an
 * execution is active. Read-only skills may intentionally have no claims.
 */
class SkillDescriptor final
{
public:
  [[nodiscard]] static SkillDescriptor create(
    std::string skill_type,
    PresetIdentity preset,
    CapabilitySet required_capabilities,
    std::vector<ResourceClaim> resource_claims,
    std::optional<PresetIdentity> deployment_identity = std::nullopt);

  [[nodiscard]] const std::string & skill_type() const noexcept;
  [[nodiscard]] const PresetIdentity & preset() const noexcept;
  [[nodiscard]] const std::optional<PresetIdentity> & deployment_identity() const noexcept;
  [[nodiscard]] const CapabilitySet & required_capabilities() const noexcept;
  [[nodiscard]] const std::vector<ResourceClaim> & resource_claims() const noexcept;

private:
  SkillDescriptor(
    std::string skill_type,
    PresetIdentity preset,
    CapabilitySet required_capabilities,
    std::vector<ResourceClaim> resource_claims,
    std::optional<PresetIdentity> deployment_identity);

  std::string skill_type_;
  PresetIdentity preset_;
  std::optional<PresetIdentity> deployment_identity_;
  CapabilitySet required_capabilities_;
  std::vector<ResourceClaim> resource_claims_;
};

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__SKILL_DESCRIPTOR_HPP_
