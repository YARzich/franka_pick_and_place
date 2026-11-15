// Responsibility: Verifies scene resource keys cannot alias across ownership scopes.

#include <gtest/gtest.h>

#include <string>

#include "manipulation/scene/resource_key.hpp"

namespace
{

using manipulation::scene::ResourceKind;

TEST(ResourceKey, ParsesCanonicalObjectKeys)
{
  const auto world = manipulation::scene::parse_resource_key("world:cube");
  ASSERT_TRUE(world);
  EXPECT_EQ(world->kind, ResourceKind::kWorldObject);
  EXPECT_EQ(world->canonical, "world:cube");
  EXPECT_EQ(world->first, "cube");
  EXPECT_TRUE(world->second.empty());

  const auto attached =
    manipulation::scene::parse_resource_key("attached:payload/fixture_1");
  ASSERT_TRUE(attached);
  EXPECT_EQ(attached->kind, ResourceKind::kAttachedObject);
  EXPECT_EQ(attached->first, "payload/fixture_1");
}

TEST(ResourceKey, RequiresCanonicalAcmOrder)
{
  const auto pair = manipulation::scene::parse_resource_key(
    "acm:left_finger|tool_mount");
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->kind, ResourceKind::kAllowedCollisionPair);
  EXPECT_EQ(pair->first, "left_finger");
  EXPECT_EQ(pair->second, "tool_mount");

  EXPECT_FALSE(manipulation::scene::parse_resource_key(
      "acm:tool_mount|left_finger"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("acm:same|same"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("acm:a|b|c"));
}

TEST(ResourceKey, RejectsMalformedOrAmbiguousKeys)
{
  EXPECT_FALSE(manipulation::scene::parse_resource_key("cube"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("world:"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("attached:a|b"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("world:two words"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("world:line\nbreak"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("acm:|right"));
  EXPECT_FALSE(manipulation::scene::parse_resource_key("acm:left|"));
}

TEST(ResourceKey, BuildersProduceCanonicalKeys)
{
  const auto world = manipulation::scene::make_world_resource_key("cube");
  ASSERT_TRUE(world);
  EXPECT_EQ(*world, "world:cube");

  const auto attached = manipulation::scene::make_attached_resource_key("cube");
  ASSERT_TRUE(attached);
  EXPECT_EQ(*attached, "attached:cube");

  const auto pair = manipulation::scene::make_acm_resource_key("z_link", "a_link");
  ASSERT_TRUE(pair);
  EXPECT_EQ(*pair, "acm:a_link|z_link");

  EXPECT_FALSE(manipulation::scene::make_world_resource_key(""));
  EXPECT_FALSE(manipulation::scene::make_acm_resource_key("same", "same"));
  EXPECT_FALSE(manipulation::scene::make_acm_resource_key("bad|link", "other"));
}

}  // namespace
