// Responsibility: Verifies immutable world snapshots validate and canonicalize rigid objects.

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/world/world_state_snapshot.hpp"

namespace
{

using manipulation::world::RigidObjectState;
using manipulation::world::WorldStateProvenance;
using manipulation::world::WorldStateSnapshot;

builtin_interfaces::msg::Time stamp(const std::int32_t seconds = 12)
{
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  result.nanosec = 345U;
  return result;
}

WorldStateProvenance provenance()
{
  return WorldStateProvenance{"perception/fused", "session-17", 42U, stamp()};
}

RigidObjectState object(
  std::string object_id,
  const double x,
  std::optional<double> confidence = std::nullopt)
{
  RigidObjectState result;
  result.object_id = std::move(object_id);
  result.pose.header.frame_id = "world";
  result.pose.header.stamp = stamp(10);
  result.pose.pose.position.x = x;
  result.pose.pose.orientation.w = 1.0;
  result.confidence = confidence;
  return result;
}

TEST(WorldStateSnapshot, OwnsProvenanceAndSortsUniqueObjects)
{
  const auto snapshot = WorldStateSnapshot::create(
    provenance(), "world/session-17/42",
    {object("tool", 0.8), object("box", 0.2, 0.75)});

  EXPECT_EQ(snapshot.provenance().source, "perception/fused");
  EXPECT_EQ(snapshot.provenance().session_id, "session-17");
  EXPECT_EQ(snapshot.provenance().revision, 42U);
  EXPECT_EQ(snapshot.provenance().capture_stamp, stamp());
  EXPECT_EQ(snapshot.token(), "world/session-17/42");
  ASSERT_EQ(snapshot.objects().size(), 2U);
  EXPECT_EQ(snapshot.objects()[0].object_id, "box");
  EXPECT_EQ(snapshot.objects()[1].object_id, "tool");
  ASSERT_TRUE(snapshot.objects()[0].confidence);
  EXPECT_DOUBLE_EQ(*snapshot.objects()[0].confidence, 0.75);
}

TEST(WorldStateSnapshot, FindsObjectsThroughCanonicalSortedIndex)
{
  const auto snapshot = WorldStateSnapshot::create(
    provenance(), "snapshot-42",
    {object("third", 3.0), object("first", 1.0), object("second", 2.0)});

  const auto * second = snapshot.find_object("second");
  ASSERT_NE(second, nullptr);
  EXPECT_DOUBLE_EQ(second->pose.pose.position.x, 2.0);
  EXPECT_EQ(snapshot.find_object("missing"), nullptr);
}

TEST(WorldStateSnapshot, AllowsAnEmptyObservedWorld)
{
  const auto snapshot = WorldStateSnapshot::create(
    provenance(), "empty-snapshot", {});

  EXPECT_TRUE(snapshot.objects().empty());
  EXPECT_EQ(snapshot.find_object("anything"), nullptr);
}

TEST(WorldStateSnapshot, RejectsMalformedProvenanceAndToken)
{
  auto invalid = provenance();
  invalid.source = "";
  EXPECT_THROW(
    WorldStateSnapshot::create(invalid, "snapshot-42", {}),
    std::invalid_argument);

  invalid = provenance();
  invalid.session_id = "two words";
  EXPECT_THROW(
    WorldStateSnapshot::create(invalid, "snapshot-42", {}),
    std::invalid_argument);

  invalid = provenance();
  invalid.capture_stamp = builtin_interfaces::msg::Time{};
  EXPECT_THROW(
    WorldStateSnapshot::create(invalid, "snapshot-42", {}),
    std::invalid_argument);

  EXPECT_THROW(
    WorldStateSnapshot::create(provenance(), "", {}),
    std::invalid_argument);
}

TEST(WorldStateSnapshot, RejectsDuplicateOrMalformedObjectIdentity)
{
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-42", {object("same", 1.0), object("same", 2.0)}),
    std::invalid_argument);
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-42", {object("bad object", 1.0)}),
    std::invalid_argument);
}

TEST(WorldStateSnapshot, RejectsInvalidStampedRigidPoses)
{
  auto invalid_frame = object("item", 1.0);
  invalid_frame.pose.header.frame_id = "/world";
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-42", {std::move(invalid_frame)}),
    std::invalid_argument);

  auto zero_stamp = object("item", 1.0);
  zero_stamp.pose.header.stamp = builtin_interfaces::msg::Time{};
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-42", {std::move(zero_stamp)}),
    std::invalid_argument);

  auto invalid_rotation = object("item", 1.0);
  invalid_rotation.pose.pose.orientation.w = 2.0;
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-42", {std::move(invalid_rotation)}),
    std::invalid_argument);
}

TEST(WorldStateSnapshot, AcceptsOnlyNormalizedFiniteConfidence)
{
  EXPECT_NO_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-zero", {object("item", 1.0, 0.0)}));
  EXPECT_NO_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-one", {object("item", 1.0, 1.0)}));
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-negative", {object("item", 1.0, -0.01)}),
    std::invalid_argument);
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-large", {object("item", 1.0, 1.01)}),
    std::invalid_argument);
  EXPECT_THROW(
    WorldStateSnapshot::create(
      provenance(), "snapshot-nan",
      {object("item", 1.0, std::numeric_limits<double>::quiet_NaN())}),
    std::invalid_argument);
}

}  // namespace
