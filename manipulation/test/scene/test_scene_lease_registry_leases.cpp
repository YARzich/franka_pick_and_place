// Responsibility: Verifies scene lease acquisition, renewal, expiry, and ownership isolation.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "manipulation/scene/scene_lease_registry.hpp"
#include "scene_lease_registry_test_support.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::scene::RegistryError;
using manipulation::scene::SceneLeaseRegistry;
using manipulation::scene::ValidateLeaseRequest;
using manipulation::scene::test::lease_request;
using manipulation::scene::test::make_registry;
using manipulation::scene::test::ManualClock;

TEST(SceneLeaseRegistry, RejectsInvalidConstruction)
{
  ManualClock clock;
  SceneLeaseRegistry::Options options;
  options.session_id = "";
  EXPECT_THROW(
    SceneLeaseRegistry(options, [&clock]() {return clock.now;}),
    std::invalid_argument);

  options.session_id = "bad session";
  EXPECT_THROW(
    SceneLeaseRegistry(options, [&clock]() {return clock.now;}),
    std::invalid_argument);

  options.session_id = "valid";
  options.max_operation_records = 0;
  EXPECT_THROW(
    SceneLeaseRegistry(options, [&clock]() {return clock.now;}),
    std::invalid_argument);

  options.max_operation_records = 1;
  EXPECT_THROW(SceneLeaseRegistry(options, {}), std::invalid_argument);
}

TEST(SceneLeaseRegistry, AcquiresResourcesAtomicallyAndExclusively)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto first = registry->acquire_lease(
    lease_request("owner-a", {"world:cube", "acm:a|b"}));
  ASSERT_TRUE(first.status.success);
  EXPECT_EQ(first.status.error, RegistryError::kNone);
  EXPECT_EQ(first.granted_ttl, 10s);
  EXPECT_NE(first.lease_token.find("test-session/lease/"), std::string::npos);

  const auto conflict = registry->acquire_lease(
    lease_request("owner-b", {"attached:free", "world:cube"}));
  EXPECT_FALSE(conflict.status.success);
  EXPECT_EQ(conflict.status.error, RegistryError::kResourceConflict);
  EXPECT_EQ(conflict.conflicting_resource_key, "world:cube");

  // Acquisition is atomic: the non-conflicting key from the failed request
  // remains available to another owner.
  const auto free = registry->acquire_lease(
    lease_request("owner-c", {"attached:free"}));
  EXPECT_TRUE(free.status.success);
  EXPECT_EQ(registry->snapshot().active_lease_count, 2U);
}

TEST(SceneLeaseRegistry, CoOwnsRelatedObjectAndAcmResourcesInOneAtomicLease)
{
  ManualClock clock;
  auto registry = make_registry(clock);

  const auto lease = registry->acquire_lease(lease_request(
      "transfer",
        {"world:cube", "attached:cube", "acm:cube|left_finger",
          "acm:cube|right_finger"}));

  EXPECT_TRUE(lease.status.success);
  EXPECT_EQ(lease.status.error, RegistryError::kNone);
  EXPECT_EQ(registry->snapshot().active_lease_count, 1U);
}

TEST(SceneLeaseRegistry, ObjectLifecycleLeaseBlocksForeignAcmAndAttachedOwnership)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  ASSERT_TRUE(registry->acquire_lease(
      lease_request("object-owner", {"world:cube"})).status.success);

  const auto acm = registry->acquire_lease(
    lease_request("acm-owner", {"acm:cube|tool"}));
  EXPECT_EQ(acm.status.error, RegistryError::kResourceConflict);
  EXPECT_EQ(acm.conflicting_resource_key, "world:cube");

  const auto attached = registry->acquire_lease(
    lease_request("attached-owner", {"attached:cube"}));
  EXPECT_EQ(attached.status.error, RegistryError::kResourceConflict);
  EXPECT_EQ(attached.conflicting_resource_key, "world:cube");
}

TEST(SceneLeaseRegistry, AcmLeaseBlocksForeignObjectOwnershipInEitherNamespace)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  ASSERT_TRUE(registry->acquire_lease(
      lease_request("acm-owner", {"acm:cube|tool"})).status.success);

  const auto world = registry->acquire_lease(
    lease_request("world-owner", {"world:cube"}));
  EXPECT_EQ(world.status.error, RegistryError::kResourceConflict);
  EXPECT_EQ(world.conflicting_resource_key, "acm:cube|tool");

  const auto attached = registry->acquire_lease(
    lease_request("attached-owner", {"attached:cube"}));
  EXPECT_EQ(attached.status.error, RegistryError::kResourceConflict);
  EXPECT_EQ(attached.conflicting_resource_key, "acm:cube|tool");
}

TEST(SceneLeaseRegistry, AllowsUnrelatedObjectsAndDistinctAcmPairs)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  ASSERT_TRUE(registry->acquire_lease(lease_request(
      "first", {"world:cube", "acm:cube|tool"})).status.success);

  EXPECT_TRUE(registry->acquire_lease(lease_request(
      "second", {"attached:payload", "acm:fixture|payload"})).status.success);
  EXPECT_TRUE(registry->acquire_lease(
      lease_request("third", {"acm:fixture|tool"})).status.success);
}

TEST(SceneLeaseRegistry, ReleaseAndAcknowledgedExpiryRestoreRelatedOwnership)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto released = registry->acquire_lease(
    lease_request("released-owner", {"world:cube"}));
  ASSERT_TRUE(released.status.success);
  ASSERT_TRUE(registry->release_lease(released.lease_token).success);

  const auto expiring = registry->acquire_lease(
    lease_request("expiring-owner", {"acm:cube|tool"}, 1s));
  ASSERT_TRUE(expiring.status.success);
  clock.advance(1s);

  const auto quarantined = registry->acquire_lease(
    lease_request("blocked-owner", {"attached:cube"}));
  EXPECT_EQ(quarantined.status.error, RegistryError::kResourceQuarantined);
  EXPECT_EQ(quarantined.conflicting_resource_key, "acm:cube|tool");

  ASSERT_TRUE(registry->rebase_session(
      "recovered-session",
      manipulation::scene::QuarantineResolution::kAccept,
      {"acm:cube|tool"}).success);
  EXPECT_TRUE(registry->acquire_lease(
      lease_request("recovered-owner", {"attached:cube"})).status.success);
}

TEST(SceneLeaseRegistry, RejectsInvalidAndDuplicateLeaseResources)
{
  ManualClock clock;
  auto registry = make_registry(clock);

  const auto malformed = registry->acquire_lease(
    lease_request("owner", {"world:not canonical"}));
  EXPECT_EQ(malformed.status.error, RegistryError::kInvalidRequest);

  const auto duplicate = registry->acquire_lease(
    lease_request("owner", {"world:cube", "world:cube"}));
  EXPECT_EQ(duplicate.status.error, RegistryError::kInvalidRequest);

  const auto empty = registry->acquire_lease(lease_request("owner", {}));
  EXPECT_EQ(empty.status.error, RegistryError::kInvalidRequest);

  const auto bad_ttl = registry->acquire_lease(
    lease_request("owner", {"world:cube"}, 0s));
  EXPECT_EQ(bad_ttl.status.error, RegistryError::kInvalidRequest);
}

TEST(SceneLeaseRegistry, RenewsFromCurrentSteadyTimeAndNeverRevivesExpiry)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto first = registry->acquire_lease(
    lease_request("owner-a", {"world:cube"}, 5s));
  ASSERT_TRUE(first.status.success);

  clock.advance(4s);
  const auto renewed = registry->renew_lease(first.lease_token, 10s);
  EXPECT_TRUE(renewed.status.success);
  EXPECT_EQ(renewed.granted_ttl, 10s);

  clock.advance(9s);
  const auto conflict = registry->acquire_lease(
    lease_request("owner-b", {"world:cube"}));
  EXPECT_EQ(conflict.status.error, RegistryError::kResourceConflict);

  clock.advance(2s);
  const auto expired = registry->renew_lease(first.lease_token, 10s);
  EXPECT_EQ(expired.status.error, RegistryError::kLeaseExpired);

  const auto second = registry->acquire_lease(
    lease_request("owner-b", {"world:cube"}));
  EXPECT_EQ(second.status.error, RegistryError::kResourceQuarantined);
  EXPECT_EQ(second.conflicting_resource_key, "world:cube");
  EXPECT_EQ(registry->snapshot().expired_lease_count, 1U);
}

TEST(SceneLeaseRegistry, ValidatesLeaseResourcesRevisionAndRemainingSteadyTtl)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request(
      "workflow", {"world:cube", "attached:cube", "acm:a|b"}, 10s));
  ASSERT_TRUE(lease.status.success);

  clock.advance(3s);
  const auto valid = registry->validate_lease(
    ValidateLeaseRequest{
      lease.lease_token, {"attached:cube", "world:cube"}, 0U});
  EXPECT_TRUE(valid.status.success);
  EXPECT_EQ(valid.status.error, RegistryError::kNone);
  EXPECT_EQ(valid.session_id, "test-session");
  EXPECT_EQ(valid.remaining_ttl, 7s);

  const auto stale_revision = registry->validate_lease(
    ValidateLeaseRequest{lease.lease_token, {"world:cube"}, 1U});
  EXPECT_EQ(stale_revision.status.error, RegistryError::kRevisionMismatch);
  EXPECT_EQ(stale_revision.status.revision, 0U);

  const auto not_owned = registry->validate_lease(
    ValidateLeaseRequest{lease.lease_token, {"world:other"}, 0U});
  EXPECT_EQ(not_owned.status.error, RegistryError::kNotOwner);
  EXPECT_EQ(not_owned.remaining_ttl, SceneLeaseRegistry::Duration::zero());
}

TEST(SceneLeaseRegistry, LeaseValidationRejectsMalformedForeignMissingAndExpiredTokens)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("workflow", {"world:cube"}, 2s));
  ASSERT_TRUE(lease.status.success);

  EXPECT_EQ(
    registry->validate_lease(
      ValidateLeaseRequest{lease.lease_token, {}, 0U}).status.error,
    RegistryError::kInvalidRequest);
  EXPECT_EQ(
    registry->validate_lease(
      ValidateLeaseRequest{
      lease.lease_token, {"world:cube", "world:cube"}, 0U}).status.error,
    RegistryError::kInvalidRequest);
  EXPECT_EQ(
    registry->validate_lease(
      ValidateLeaseRequest{
      "other-session/lease/1", {"world:cube"}, 0U}).status.error,
    RegistryError::kSessionMismatch);
  EXPECT_EQ(
    registry->validate_lease(
      ValidateLeaseRequest{
      "test-session/lease/999", {"world:cube"}, 0U}).status.error,
    RegistryError::kLeaseNotFound);

  clock.advance(2s);
  const auto expired = registry->validate_lease(
    ValidateLeaseRequest{lease.lease_token, {"world:cube"}, 0U});
  EXPECT_EQ(expired.status.error, RegistryError::kLeaseExpired);
  EXPECT_EQ(expired.session_id, "test-session");
  EXPECT_EQ(registry->snapshot().active_lease_count, 0U);
}

TEST(SceneLeaseRegistry, ExplicitExpiryReportsRemovedLeaseTokens)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}, 2s));
  ASSERT_TRUE(lease.status.success);

  clock.advance(2s);
  const auto expired = registry->expire_leases();
  ASSERT_EQ(expired.size(), 1U);
  EXPECT_EQ(expired.front(), lease.lease_token);
  EXPECT_EQ(registry->snapshot().active_lease_count, 0U);
  EXPECT_EQ(
    registry->snapshot().quarantined_resource_keys,
    std::vector<std::string>{"world:cube"});
  EXPECT_EQ(registry->snapshot().expired_lease_count, 1U);
  EXPECT_TRUE(registry->expire_leases().empty());
}

TEST(SceneLeaseRegistry, ExpiryQuarantinesResourcesUntilExplicitSessionRebase)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("unsafe-owner", {"world:cube", "attached:cube"}, 1s));
  ASSERT_TRUE(lease.status.success);

  clock.advance(1s);
  EXPECT_EQ(
    registry->rebase_session("unacknowledged-session").error,
    RegistryError::kResourceQuarantined);
  const auto quarantined = registry->snapshot();
  EXPECT_EQ(
    quarantined.quarantined_resource_keys,
    (std::vector<std::string>{"attached:cube", "world:cube"}));

  const auto world_conflict = registry->acquire_lease(
    lease_request("recovery-race", {"world:cube"}));
  EXPECT_EQ(world_conflict.status.error, RegistryError::kResourceQuarantined);
  EXPECT_EQ(world_conflict.conflicting_resource_key, "attached:cube");
  EXPECT_EQ(
    registry->acquire_lease(
      lease_request("recovery-race", {"attached:cube"})).status.error,
    RegistryError::kResourceQuarantined);

  EXPECT_EQ(
    registry->rebase_session(
      "stale-recovery-session",
      manipulation::scene::QuarantineResolution::kAccept,
      {"world:cube"}).error,
    RegistryError::kResourceQuarantined);
  ASSERT_TRUE(registry->rebase_session(
      "recovered-session",
      manipulation::scene::QuarantineResolution::kAccept,
      {"attached:cube", "world:cube"}).success);
  EXPECT_TRUE(registry->snapshot().quarantined_resource_keys.empty());
  EXPECT_TRUE(registry->acquire_lease(
      lease_request("recovered-owner", {"world:cube", "attached:cube"})).status.success);
}

TEST(SceneLeaseRegistry, ReleaseChecksSessionAndMakesResourcesAvailable)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner-a", {"world:cube"}));
  ASSERT_TRUE(lease.status.success);

  const auto foreign = registry->release_lease("other-session/lease/1");
  EXPECT_EQ(foreign.error, RegistryError::kSessionMismatch);
  const auto missing = registry->release_lease("test-session/lease/999");
  EXPECT_EQ(missing.error, RegistryError::kLeaseNotFound);

  EXPECT_TRUE(registry->release_lease(lease.lease_token).success);
  EXPECT_TRUE(registry->acquire_lease(
      lease_request("owner-b", {"world:cube"})).status.success);
}

TEST(SceneLeaseRegistry, ConcurrentAcquireHasExactlyOneWinner)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  std::promise<void> start_promise;
  auto start = start_promise.get_future().share();
  std::atomic<int> successful_acquisitions{0};

  const auto acquire = [&](const std::string & owner) {
      start.wait();
      if (registry->acquire_lease(
          lease_request(owner, {"world:cube"})).status.success)
      {
        ++successful_acquisitions;
      }
    };

  std::thread first(acquire, "owner-a");
  std::thread second(acquire, "owner-b");
  start_promise.set_value();
  first.join();
  second.join();

  EXPECT_EQ(successful_acquisitions.load(), 1);
  EXPECT_EQ(registry->snapshot().active_lease_count, 1U);
}

}  // namespace
