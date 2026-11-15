// Responsibility: Verifies normalized skill metadata keeps distinct capability and ownership roles.

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "manipulation/skill/skill_descriptor.hpp"
#include "manipulation/skill/standard_capabilities.hpp"

namespace
{

TEST(SkillDescriptor, OwnsCanonicalMetadataAndDeduplicatedClaims)
{
  const auto preset = manipulation::skill::PresetIdentity::from_canonical_content(
    "tabletop/transfer", 1U, "velocity=0.4");
  const auto deployment = manipulation::skill::PresetIdentity::from_canonical_content(
    "moveit/transfer_process", 1U, "timeout=30");
  const auto descriptor = manipulation::skill::SkillDescriptor::create(
    "transfer/object",
    preset,
    manipulation::skill::CapabilitySet{
      std::string{manipulation::skill::capability::kPlanExecutePose},
      std::string{manipulation::skill::capability::kGripperWidthCommand}},
    {manipulation::skill::ResourceClaim{"gripper/main"},
      manipulation::skill::ResourceClaim{"arm/main"},
      manipulation::skill::ResourceClaim{"arm/main"}},
    deployment);

  EXPECT_EQ(descriptor.skill_type(), "transfer/object");
  EXPECT_EQ(descriptor.preset(), preset);
  ASSERT_TRUE(descriptor.deployment_identity());
  EXPECT_EQ(*descriptor.deployment_identity(), deployment);
  EXPECT_TRUE(descriptor.required_capabilities().contains(
      manipulation::skill::capability::kPlanExecutePose));
  ASSERT_EQ(descriptor.resource_claims().size(), 2U);
  EXPECT_EQ(descriptor.resource_claims()[0].resource_id(), "arm/main");
  EXPECT_EQ(descriptor.resource_claims()[1].resource_id(), "gripper/main");
}

TEST(SkillDescriptor, AllowsReadOnlySkillWithoutPhysicalClaims)
{
  const auto preset = manipulation::skill::PresetIdentity::from_canonical_content(
    "inspection/default", 1U, "frame=world");
  const auto descriptor = manipulation::skill::SkillDescriptor::create(
    "inspection/object", preset,
    manipulation::skill::CapabilitySet{"world/rigid_object"}, {});

  EXPECT_TRUE(descriptor.resource_claims().empty());
}

TEST(SkillDescriptor, RejectsMalformedSkillType)
{
  const auto preset = manipulation::skill::PresetIdentity::from_canonical_content(
    "inspection/default", 1U, "frame=world");
  EXPECT_THROW(
    manipulation::skill::SkillDescriptor::create(
      "Transfer Object", preset, manipulation::skill::CapabilitySet{}, {}),
    std::invalid_argument);
}

}  // namespace
