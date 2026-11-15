// Responsibility: Verifies shared sequence reservation exclusivity and RAII lifetime.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "manipulation/workflow/transfer_sequence_goal_reservation.hpp"

namespace manipulation::workflow
{
namespace
{

static_assert(!std::is_copy_constructible_v<TransferSequenceGoalReservation>);
static_assert(!std::is_copy_assignable_v<TransferSequenceGoalReservation>);
static_assert(!std::is_move_constructible_v<TransferSequenceGoalReservation>);
static_assert(!std::is_move_assignable_v<TransferSequenceGoalReservation>);
static_assert(!std::is_copy_constructible_v<TransferSequenceGoalReservation::Lease>);
static_assert(!std::is_copy_assignable_v<TransferSequenceGoalReservation::Lease>);
static_assert(std::is_nothrow_move_constructible_v<TransferSequenceGoalReservation::Lease>);
static_assert(std::is_nothrow_move_assignable_v<TransferSequenceGoalReservation::Lease>);
static_assert(std::is_nothrow_destructible_v<TransferSequenceGoalReservation::Lease>);

TEST(TransferSequenceGoalReservation, GrantsOneLeaseAndRejectsConcurrentOwner)
{
  TransferSequenceGoalReservation reservation;

  auto first = reservation.try_acquire("static_sequence");
  ASSERT_TRUE(first.has_value());
  EXPECT_TRUE(first->valid());
  EXPECT_EQ(first->generation(), 1U);
  EXPECT_FALSE(reservation.try_acquire("perception_sequence").has_value());

  const auto occupied = reservation.snapshot();
  EXPECT_TRUE(occupied.reserved);
  EXPECT_EQ(occupied.generation, 1U);
  EXPECT_EQ(occupied.owner, "static_sequence");

  first.reset();
  const auto released = reservation.snapshot();
  EXPECT_FALSE(released.reserved);
  EXPECT_EQ(released.generation, 1U);
  EXPECT_TRUE(released.owner.empty());

  auto second = reservation.try_acquire("perception_sequence");
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->generation(), 2U);
  EXPECT_EQ(reservation.snapshot().owner, "perception_sequence");
}

TEST(TransferSequenceGoalReservation, MoveTransfersOwnershipAndMoveAssignmentReleasesOldLease)
{
  TransferSequenceGoalReservation first_reservation;
  TransferSequenceGoalReservation second_reservation;
  auto first = first_reservation.try_acquire("first");
  auto second = second_reservation.try_acquire("second");
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  TransferSequenceGoalReservation::Lease moved{std::move(*first)};
  EXPECT_FALSE(first->valid());
  EXPECT_EQ(first->generation(), 0U);
  EXPECT_TRUE(moved.valid());
  EXPECT_TRUE(first_reservation.snapshot().reserved);

  moved = std::move(*second);
  EXPECT_FALSE(second->valid());
  EXPECT_FALSE(first_reservation.snapshot().reserved);
  EXPECT_TRUE(second_reservation.snapshot().reserved);

  moved = std::move(moved);
  EXPECT_TRUE(moved.valid());
  EXPECT_TRUE(second_reservation.snapshot().reserved);
}

TEST(TransferSequenceGoalReservation, LeaseCanOutliveReservationWrapper)
{
  std::optional<TransferSequenceGoalReservation::Lease> surviving_lease;
  {
    auto reservation = std::make_unique<TransferSequenceGoalReservation>();
    surviving_lease = reservation->try_acquire("long_lived_goal");
    ASSERT_TRUE(surviving_lease.has_value());
    reservation.reset();
  }

  EXPECT_TRUE(surviving_lease->valid());
  EXPECT_EQ(surviving_lease->generation(), 1U);
  EXPECT_NO_THROW(surviving_lease.reset());
}

TEST(TransferSequenceGoalReservation, SimultaneousAcquireHasExactlyOneWinner)
{
  constexpr std::size_t contender_count = 16U;
  TransferSequenceGoalReservation reservation;
  std::atomic<std::size_t> ready_count{0U};
  std::promise<void> all_ready;
  auto all_ready_future = all_ready.get_future();
  std::promise<void> start;
  const auto start_gate = start.get_future().share();
  std::atomic<std::size_t> winner_count{0U};
  std::atomic<std::size_t> attempted_count{0U};
  std::promise<void> all_attempted;
  auto all_attempted_future = all_attempted.get_future();
  std::promise<void> release_winner;
  const auto release_gate = release_winner.get_future().share();
  std::vector<std::thread> contenders;
  contenders.reserve(contender_count);

  for (std::size_t index = 0U; index < contender_count; ++index) {
    contenders.emplace_back(
      [&, index]() {
        if (ready_count.fetch_add(1U) + 1U == contender_count) {
          all_ready.set_value();
        }
        start_gate.wait();
        auto lease = reservation.try_acquire("contender_" + std::to_string(index));
        if (lease) {
          ++winner_count;
        }
        if (attempted_count.fetch_add(1U) + 1U == contender_count) {
          all_attempted.set_value();
        }
        if (lease) {
          release_gate.wait();
        }
      });
  }

  const auto ready_status = all_ready_future.wait_for(std::chrono::seconds{5});
  start.set_value();
  const auto attempted_status = all_attempted_future.wait_for(std::chrono::seconds{5});
  const auto occupied = reservation.snapshot();

  release_winner.set_value();
  for (auto & contender : contenders) {
    contender.join();
  }
  EXPECT_EQ(ready_status, std::future_status::ready);
  EXPECT_EQ(attempted_status, std::future_status::ready);
  EXPECT_EQ(winner_count.load(), 1U);
  EXPECT_TRUE(occupied.reserved);
  EXPECT_FALSE(reservation.snapshot().reserved);
}

TEST(TransferSequenceGoalReservation, RejectsEmptyAndWhitespaceOnlyOwner)
{
  TransferSequenceGoalReservation reservation;

  EXPECT_THROW((void)reservation.try_acquire(""), std::invalid_argument);
  EXPECT_THROW((void)reservation.try_acquire(" \t\n\r"), std::invalid_argument);
  EXPECT_NO_THROW((void)reservation.try_acquire(" valid owner "));
}

}  // namespace
}  // namespace manipulation::workflow
