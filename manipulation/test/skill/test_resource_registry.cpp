// Responsibility: Verifies atomic, RAII, generation-safe logical resource ownership.

#include <atomic>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "manipulation/skill/resource_claim.hpp"
#include "manipulation/skill/resource_registry.hpp"

namespace manipulation::skill
{
namespace
{

TEST(ResourceClaim, RejectsMalformedLogicalResourceId)
{
  EXPECT_NO_THROW(ResourceClaim("arm/main"));
  EXPECT_THROW(ResourceClaim("arm//main"), std::invalid_argument);
  EXPECT_THROW(ResourceClaim("Arm/main"), std::invalid_argument);
}

TEST(ResourceRegistry, AcquiresMultipleClaimsAtomicallyAndNormalizesDuplicates)
{
  ResourceRegistry registry;
  auto acquired = registry.try_acquire(
    "skill/transfer_object",
    {ResourceClaim("gripper/main"), ResourceClaim("arm/main"),
      ResourceClaim("arm/main")});

  ASSERT_TRUE(acquired.acquired());
  ASSERT_TRUE(acquired.lease.has_value());
  EXPECT_EQ(
    acquired.lease->resource_ids(),
    (std::vector<std::string>{"arm/main", "gripper/main"}));
  const auto snapshot = registry.snapshot();
  ASSERT_EQ(snapshot.active_ownerships.size(), 2U);
  EXPECT_EQ(snapshot.active_ownerships[0].resource_id, "arm/main");
  EXPECT_EQ(snapshot.active_ownerships[1].resource_id, "gripper/main");
  EXPECT_EQ(snapshot.active_ownerships[0].generation, acquired.lease->generation());
  EXPECT_EQ(snapshot.active_ownerships[1].generation, acquired.lease->generation());
}

TEST(ResourceRegistry, RejectsWholeRequestWhenOneResourceConflicts)
{
  ResourceRegistry registry;
  auto first = registry.try_acquire(
    "skill/transfer_object", {ResourceClaim("arm/main")});
  ASSERT_TRUE(first.acquired());
  const auto before_conflict = registry.snapshot();
  ASSERT_EQ(before_conflict.active_ownerships.size(), 1U);

  auto blocked = registry.try_acquire(
    "skill/servo_to_pose",
    {ResourceClaim("arm/main"), ResourceClaim("camera/wrist")});

  EXPECT_EQ(blocked.error, ResourceAcquireError::kConflict);
  EXPECT_EQ(blocked.conflicting_resource_id, "arm/main");
  EXPECT_FALSE(blocked.lease.has_value());
  const auto snapshot = registry.snapshot();
  EXPECT_EQ(snapshot.last_generation, before_conflict.last_generation);
  ASSERT_EQ(snapshot.active_ownerships.size(), 1U);
  EXPECT_EQ(snapshot.active_ownerships.front().resource_id, "arm/main");
  EXPECT_EQ(
    snapshot.active_ownerships.front().generation,
    before_conflict.active_ownerships.front().generation);

  first.lease.reset();
  auto next = registry.try_acquire(
    "skill/servo_to_pose", {ResourceClaim("arm/main")});
  ASSERT_TRUE(next.acquired());
  EXPECT_EQ(next.lease->generation(), before_conflict.last_generation + 1U);
}

TEST(ResourceRegistry, ValidatesOwnerAndRequiresAtLeastOneClaim)
{
  ResourceRegistry registry;
  auto invalid_owner = registry.try_acquire(
    "Servo Goal", {ResourceClaim("arm/main")});
  EXPECT_EQ(invalid_owner.error, ResourceAcquireError::kInvalidOwner);

  auto empty = registry.try_acquire("skill/servo_to_pose", {});
  EXPECT_EQ(empty.error, ResourceAcquireError::kEmptyClaims);
  EXPECT_TRUE(registry.snapshot().active_ownerships.empty());
}

TEST(ResourceRegistry, LeaseMoveAndDestructionReleaseOnlyOwnedGeneration)
{
  ResourceRegistry registry;
  std::uint64_t first_generation = 0U;
  {
    auto result = registry.try_acquire(
      "skill/first", {ResourceClaim("arm/main")});
    ASSERT_TRUE(result.acquired());
    first_generation = result.lease->generation();

    auto moved = std::move(*result.lease);
    EXPECT_FALSE(result.lease->valid());
    EXPECT_TRUE(moved.valid());
  }
  EXPECT_TRUE(registry.snapshot().active_ownerships.empty());

  auto second = registry.try_acquire(
    "skill/second", {ResourceClaim("arm/main")});
  ASSERT_TRUE(second.acquired());
  EXPECT_GT(second.lease->generation(), first_generation);
}

TEST(ResourceRegistry, LeaseSafelyOutlivesRegistryWrapper)
{
  std::optional<ResourceRegistry::Lease> retained;
  {
    auto registry = std::make_unique<ResourceRegistry>();
    auto result = registry->try_acquire(
      "skill/transfer_object", {ResourceClaim("arm/main")});
    ASSERT_TRUE(result.acquired());
    retained.emplace(std::move(*result.lease));
  }

  EXPECT_TRUE(retained->valid());
  retained.reset();
}

TEST(ResourceRegistry, ConcurrentConflictHasExactlyOneWinner)
{
  ResourceRegistry registry;
  std::atomic<bool> start{false};
  std::atomic<int> winner_count{0};
  std::optional<ResourceRegistry::Lease> first_lease;
  std::optional<ResourceRegistry::Lease> second_lease;

  auto compete = [&](const std::string & owner,
    std::optional<ResourceRegistry::Lease> & lease) {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      auto result = registry.try_acquire(owner, {ResourceClaim("arm/main")});
      if (result.acquired()) {
        ++winner_count;
        lease.emplace(std::move(*result.lease));
      }
    };

  std::thread first(compete, "skill/first", std::ref(first_lease));
  std::thread second(compete, "skill/second", std::ref(second_lease));
  start.store(true, std::memory_order_release);
  first.join();
  second.join();

  EXPECT_EQ(winner_count.load(), 1);
  EXPECT_NE(first_lease.has_value(), second_lease.has_value());
  EXPECT_EQ(registry.snapshot().active_ownerships.size(), 1U);
}

}  // namespace
}  // namespace manipulation::skill
