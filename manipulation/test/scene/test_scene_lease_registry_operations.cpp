// Responsibility: Verifies revisioned scene operations, retry identity, and mutation quarantine.

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "manipulation/scene/scene_lease_registry.hpp"
#include "scene_lease_registry_test_support.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::scene::BackendOperationResult;
using manipulation::scene::OperationDisposition;
using manipulation::scene::RegistryError;
using manipulation::scene::ValidateLeaseRequest;
using manipulation::scene::test::lease_request;
using manipulation::scene::test::make_registry;
using manipulation::scene::test::ManualClock;
using manipulation::scene::test::operation_request;

TEST(SceneLeaseRegistry, SuccessfulBackendMutationAdvancesRevisionExactlyOnce)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube", "attached:cube"}));
  ASSERT_TRUE(lease.status.success);

  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "add-cube", 0, {"world:cube"}, "sha256:add"));
  ASSERT_TRUE(begin.status.success);
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);
  ASSERT_FALSE(begin.operation_ticket.empty());

  const auto completed = registry->complete_operation(
    begin.operation_ticket, BackendOperationResult{true, "cube added"});
  EXPECT_TRUE(completed.success);
  EXPECT_EQ(completed.revision, 1U);
  EXPECT_EQ(registry->snapshot().revision, 1U);

  const auto second_completion = registry->complete_operation(
    begin.operation_ticket, BackendOperationResult{true, {}});
  EXPECT_EQ(second_completion.error, RegistryError::kOperationTicketInvalid);
  EXPECT_EQ(second_completion.revision, 1U);

  const auto stale = registry->begin_operation(
    operation_request(lease.lease_token, "attach-cube", 0, {"attached:cube"}, "sha256:attach"));
  EXPECT_EQ(stale.status.error, RegistryError::kRevisionMismatch);
  EXPECT_EQ(stale.status.revision, 1U);
}

TEST(SceneLeaseRegistry, FailedBackendMutationIsStableAndDoesNotAdvanceRevision)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  ASSERT_TRUE(lease.status.success);
  const auto request = operation_request(
    lease.lease_token, "add-cube", 0, {"world:cube"}, "sha256:add");
  const auto begin = registry->begin_operation(request);
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  const auto failed = registry->complete_operation(
    begin.operation_ticket, BackendOperationResult{false, "MoveIt rejected diff"});
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(failed.error, RegistryError::kBackendFailed);
  EXPECT_EQ(failed.revision, 0U);

  const auto replay = registry->begin_operation(request);
  EXPECT_EQ(replay.disposition, OperationDisposition::kReplay);
  EXPECT_FALSE(replay.status.success);
  EXPECT_EQ(replay.status.error, RegistryError::kBackendFailed);
  EXPECT_EQ(replay.status.message, "MoveIt rejected diff");
  EXPECT_EQ(registry->snapshot().revision, 0U);
}

TEST(SceneLeaseRegistry, TimeoutOutcomeRemainsTypedAcrossReplay)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto request = operation_request(
    lease.lease_token, "timed-out", 0, {"world:cube"}, "digest");
  const auto begin = registry->begin_operation(request);
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  const auto timeout = registry->complete_operation(
    begin.operation_ticket,
    BackendOperationResult{false, "backend response timeout", RegistryError::kTimeout});
  EXPECT_EQ(timeout.error, RegistryError::kTimeout);
  EXPECT_EQ(timeout.revision, 0U);

  const auto replay = registry->begin_operation(request);
  EXPECT_EQ(replay.disposition, OperationDisposition::kReplay);
  EXPECT_EQ(replay.status.error, RegistryError::kTimeout);
  EXPECT_EQ(replay.status.message, "backend response timeout");
}

TEST(SceneLeaseRegistry, EnforcesOwnershipForEveryOperationResource)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto first = registry->acquire_lease(
    lease_request("owner-a", {"world:a"}));
  const auto second = registry->acquire_lease(
    lease_request("owner-b", {"world:b"}));
  ASSERT_TRUE(first.status.success);
  ASSERT_TRUE(second.status.success);

  const auto unauthorized = registry->begin_operation(
    operation_request(first.lease_token, "modify-b", 0, {"world:b"}, "sha256:b"));
  EXPECT_EQ(unauthorized.status.error, RegistryError::kNotOwner);
  EXPECT_EQ(unauthorized.disposition, OperationDisposition::kRejected);
}

TEST(SceneLeaseRegistry, ReplaysOnlyTheExactSameRequest)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube", "acm:a|b"}));
  ASSERT_TRUE(lease.status.success);
  const auto request = operation_request(
    lease.lease_token,
    "operation-uuid",
    0,
    {"world:cube", "acm:a|b"},
    "sha256:original");

  const auto begin = registry->begin_operation(request);
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  auto same_reordered = request;
  same_reordered.resource_keys = {"acm:a|b", "world:cube"};
  const auto in_progress = registry->begin_operation(same_reordered);
  EXPECT_EQ(in_progress.status.error, RegistryError::kOperationInProgress);

  auto conflicting = request;
  conflicting.request_digest = "sha256:different";
  const auto conflict = registry->begin_operation(conflicting);
  EXPECT_EQ(conflict.status.error, RegistryError::kIdempotencyConflict);

  ASSERT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{true, {}}).success);

  const auto replay = registry->begin_operation(same_reordered);
  EXPECT_EQ(replay.disposition, OperationDisposition::kReplay);
  EXPECT_TRUE(replay.status.success);
  EXPECT_EQ(replay.status.revision, 1U);
  EXPECT_TRUE(replay.operation_ticket.empty());

  conflicting.expected_revision = 1;
  const auto conflicting_replay = registry->begin_operation(conflicting);
  EXPECT_EQ(conflicting_replay.status.error, RegistryError::kIdempotencyConflict);
}

TEST(SceneLeaseRegistry, SerializesDifferentOperationsAcrossOwners)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto first = registry->acquire_lease(
    lease_request("owner-a", {"world:a"}));
  const auto second = registry->acquire_lease(
    lease_request("owner-b", {"world:b"}));
  ASSERT_TRUE(first.status.success);
  ASSERT_TRUE(second.status.success);

  const auto active = registry->begin_operation(
    operation_request(first.lease_token, "op-a", 0, {"world:a"}, "digest-a"));
  ASSERT_EQ(active.disposition, OperationDisposition::kReady);
  const auto blocked = registry->begin_operation(
    operation_request(second.lease_token, "op-b", 0, {"world:b"}, "digest-b"));
  EXPECT_EQ(blocked.status.error, RegistryError::kOperationInProgress);

  ASSERT_TRUE(registry->complete_operation(
      active.operation_ticket, BackendOperationResult{true, {}}).success);
  const auto next = registry->begin_operation(
    operation_request(second.lease_token, "op-b", 1, {"world:b"}, "digest-b"));
  EXPECT_EQ(next.disposition, OperationDisposition::kReady);
}

TEST(SceneLeaseRegistry, LeaseValidationFailsWhileMutationIsInFlight)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  ASSERT_TRUE(lease.status.success);
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "op", 0, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  const auto in_flight = registry->validate_lease(
    ValidateLeaseRequest{lease.lease_token, {"world:cube"}, 0U});
  EXPECT_EQ(in_flight.status.error, RegistryError::kOperationInProgress);

  ASSERT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{true, {}}).success);
  EXPECT_EQ(
    registry->validate_lease(
      ValidateLeaseRequest{lease.lease_token, {"world:cube"}, 0U}).status.error,
    RegistryError::kRevisionMismatch);
  EXPECT_TRUE(registry->validate_lease(
      ValidateLeaseRequest{lease.lease_token, {"world:cube"}, 1U}).status.success);
}

TEST(SceneLeaseRegistry, InFlightOperationPinsAnExpiredLeaseUntilCompletion)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner-a", {"world:cube"}, 1s));
  ASSERT_TRUE(lease.status.success);
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "slow-op", 0, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  clock.advance(2s);
  EXPECT_TRUE(registry->expire_leases().empty());
  EXPECT_EQ(registry->snapshot().active_lease_count, 1U);
  EXPECT_EQ(
    registry->renew_lease(lease.lease_token, 10s).status.error,
    RegistryError::kLeaseExpired);
  EXPECT_EQ(
    registry->release_lease(lease.lease_token).error,
    RegistryError::kOperationInProgress);
  EXPECT_EQ(
    registry->acquire_lease(
      lease_request("owner-b", {"world:cube"})).status.error,
    RegistryError::kResourceConflict);

  ASSERT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{true, {}}).success);
  EXPECT_EQ(registry->snapshot().active_lease_count, 0U);
  EXPECT_EQ(
    registry->acquire_lease(
      lease_request("owner-b", {"world:cube"})).status.error,
    RegistryError::kResourceQuarantined);
}

TEST(SceneLeaseRegistry, InvalidTicketDoesNotAbandonTheActiveOperation)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "op", 0, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  EXPECT_EQ(
    registry->complete_operation("test-session/operation/wrong", {true, {}}).error,
    RegistryError::kOperationTicketInvalid);
  EXPECT_TRUE(registry->snapshot().operation_in_progress);
  EXPECT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{false, "aborted"}).error ==
    RegistryError::kBackendFailed);
  EXPECT_FALSE(registry->snapshot().operation_in_progress);
}

TEST(SceneLeaseRegistry, RejectsNewOperationIdsAtConfiguredCapacity)
{
  ManualClock clock;
  auto registry = make_registry(clock, 1);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto first = registry->begin_operation(
    operation_request(lease.lease_token, "first", 0, {"world:cube"}, "digest-1"));
  ASSERT_EQ(first.disposition, OperationDisposition::kReady);
  ASSERT_TRUE(registry->complete_operation(
      first.operation_ticket, BackendOperationResult{true, {}}).success);

  const auto second = registry->begin_operation(
    operation_request(lease.lease_token, "second", 1, {"world:cube"}, "digest-2"));
  EXPECT_EQ(second.status.error, RegistryError::kOperationCapacityExceeded);
  const auto full_snapshot = registry->snapshot();
  EXPECT_EQ(full_snapshot.operation_record_capacity, 1U);
  EXPECT_TRUE(full_snapshot.operation_capacity_exhausted);

  // Existing records remain replayable after capacity is reached.
  const auto replay = registry->begin_operation(
    operation_request(lease.lease_token, "first", 0, {"world:cube"}, "digest-1"));
  EXPECT_EQ(replay.disposition, OperationDisposition::kReplay);
}

TEST(SceneLeaseRegistry, PreservesConfiguredInitialRevision)
{
  ManualClock clock;
  auto registry = make_registry(clock, 4, 42);
  EXPECT_EQ(registry->snapshot().revision, 42U);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "op", 42, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);
  EXPECT_EQ(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{true, {}}).revision, 43U);
}

TEST(SceneLeaseRegistry, RebaseInvalidatesLeasesAndLedgerWithoutReusingSession)
{
  ManualClock clock;
  auto registry = make_registry(clock, 4, 3);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "old-op", 3, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);
  ASSERT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{true, {}}).success);

  const auto reset = registry->rebase_session("replacement-session");
  ASSERT_TRUE(reset.success);
  EXPECT_EQ(reset.revision, 4U);
  const auto snapshot = registry->snapshot();
  EXPECT_EQ(snapshot.session_id, "replacement-session");
  EXPECT_EQ(snapshot.revision, 4U);
  EXPECT_EQ(snapshot.active_lease_count, 0U);
  EXPECT_EQ(snapshot.operation_record_count, 0U);
  EXPECT_EQ(
    registry->release_lease(lease.lease_token).error,
    RegistryError::kSessionMismatch);
}

TEST(SceneLeaseRegistry, RebaseRejectsAnActiveOperation)
{
  ManualClock clock;
  auto registry = make_registry(clock);
  const auto lease = registry->acquire_lease(
    lease_request("owner", {"world:cube"}));
  const auto begin = registry->begin_operation(
    operation_request(lease.lease_token, "op", 0, {"world:cube"}, "digest"));
  ASSERT_EQ(begin.disposition, OperationDisposition::kReady);

  EXPECT_EQ(
    registry->rebase_session("replacement-session").error,
    RegistryError::kOperationInProgress);
  EXPECT_TRUE(registry->complete_operation(
      begin.operation_ticket, BackendOperationResult{false, "stopped"}).error ==
    RegistryError::kBackendFailed);
}

}  // namespace
