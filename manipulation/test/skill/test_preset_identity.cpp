// Responsibility: Verifies immutable preset identity validation and deterministic digesting.

#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "manipulation/skill/preset_identity.hpp"

namespace
{

TEST(PresetIdentity, ProducesStableDigestForCanonicalContent)
{
  const auto first = manipulation::skill::PresetIdentity::from_canonical_content(
    "tabletop/cube_transfer", 1U, "speed=0.5\nclearance=0.1\n");
  const auto second = manipulation::skill::PresetIdentity::from_canonical_content(
    "tabletop/cube_transfer", 1U, "speed=0.5\nclearance=0.1\n");

  EXPECT_EQ(first, second);
  EXPECT_EQ(first.preset_id(), "tabletop/cube_transfer");
  EXPECT_EQ(first.schema_version(), 1U);
  EXPECT_EQ(first.canonical_digest().size(), 64U);
}

TEST(PresetIdentity, ContentChangeChangesDigest)
{
  const auto first = manipulation::skill::PresetIdentity::from_canonical_content(
    "tabletop/cube_transfer", 1U, "speed=0.5");
  const auto second = manipulation::skill::PresetIdentity::from_canonical_content(
    "tabletop/cube_transfer", 1U, "speed=0.6");

  EXPECT_NE(first, second);
}

TEST(PresetIdentity, ReconstructsValidatedIdentityFromDigest)
{
  const std::string digest(64U, 'a');
  const auto identity = manipulation::skill::PresetIdentity::from_digest(
    "mobile/pick", 2U, digest);

  EXPECT_EQ(identity.canonical_digest(), digest);
}

TEST(PresetIdentity, RejectsInvalidIdentityFields)
{
  const std::string valid_digest(64U, '0');
  EXPECT_THROW(
    manipulation::skill::PresetIdentity::from_digest("Bad ID", 1U, valid_digest),
    std::invalid_argument);
  EXPECT_THROW(
    manipulation::skill::PresetIdentity::from_digest("tabletop/pick", 0U, valid_digest),
    std::invalid_argument);
  EXPECT_THROW(
    manipulation::skill::PresetIdentity::from_digest(
      "tabletop/pick", 1U, std::string(64U, 'A')),
    std::invalid_argument);
}

}  // namespace
