// Responsibility: Defines an exclusive claim for one canonical logical control resource.

#ifndef MANIPULATION__SKILL__RESOURCE_CLAIM_HPP_
#define MANIPULATION__SKILL__RESOURCE_CLAIM_HPP_

#include <string>

namespace manipulation::skill
{

/**
 * Immutable exclusive claim such as `arm/main` or `gripper/main`.
 *
 * Logical identity is deliberately independent from a ROS topic, action, or
 * controller name: deployments may expose aliases that still address the same
 * physical resource.
 */
class ResourceClaim final
{
public:
  explicit ResourceClaim(std::string resource_id);

  [[nodiscard]] const std::string & resource_id() const noexcept;

  [[nodiscard]] bool operator==(const ResourceClaim & other) const noexcept;
  [[nodiscard]] bool operator!=(const ResourceClaim & other) const noexcept;
  [[nodiscard]] bool operator<(const ResourceClaim & other) const noexcept;

private:
  std::string resource_id_;
};

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__RESOURCE_CLAIM_HPP_
