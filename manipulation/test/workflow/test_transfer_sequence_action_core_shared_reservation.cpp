// Responsibility: Verifies cross-facade goal exclusion across the neutral lifecycle core.

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>

#include "transfer_sequence_action_core_test_support.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using namespace std::chrono_literals;
using test::TransferSequenceActionCoreTest;
using test::accepted_admission;
using test::descriptor;
using test::goal_id;
using test::successful_execution;
using test::transport_port;

TEST_F(TransferSequenceActionCoreTest, SharedReservationRejectsBeforeLoserAdmission)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto first_core = make_core("shared_first", {}, {}, shared_reservation);
  auto second_core = make_core("shared_second", {}, {}, shared_reservation);
  std::atomic_uint loser_admission_count{0U};

  ASSERT_EQ(
    first_core->handle_goal(
      goal_id(1U), descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  ASSERT_TRUE(shared_reservation->snapshot().reserved);

  EXPECT_EQ(
    second_core->handle_goal(
      goal_id(21U), descriptor(),
      [&loser_admission_count]() {
        ++loser_admission_count;
        return accepted_admission();
      },
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::REJECT);
  EXPECT_EQ(loser_admission_count.load(), 0U);
}

TEST_F(TransferSequenceActionCoreTest, BackendAdmissionRejectionReleasesSharedReservation)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_reject", {}, {}, shared_reservation);

  EXPECT_EQ(
    core->handle_goal(
      goal_id(41U), descriptor(),
      []() {
        return TransferSequenceAdmission{
          false, manipulation::task::ObjectDisposition::kUnknown,
          "intentional test rejection"};
      },
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::REJECT);
  EXPECT_FALSE(shared_reservation->snapshot().reserved);
}

TEST_F(TransferSequenceActionCoreTest, BackendAdmissionExceptionReleasesSharedReservation)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_exception", {}, {}, shared_reservation);

  EXPECT_EQ(
    core->handle_goal(
      goal_id(61U), descriptor(),
      []() -> TransferSequenceAdmission {
        throw std::runtime_error("intentional test exception");
      },
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::REJECT);
  EXPECT_FALSE(shared_reservation->snapshot().reserved);
}

TEST_F(TransferSequenceActionCoreTest, AdmissionDeadlineReleasesSharedReservation)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_deadline", {}, {}, shared_reservation);
  auto goal = descriptor();
  goal.requested_timeout = 1ms;

  EXPECT_EQ(
    core->handle_goal(
      goal_id(81U), goal,
      []() {
        std::this_thread::sleep_for(5ms);
        return accepted_admission();
      },
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::REJECT);
  EXPECT_FALSE(shared_reservation->snapshot().reserved);
}

TEST_F(TransferSequenceActionCoreTest, DestructionBeforeAcceptanceReleasesSharedReservation)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_unaccepted", {}, {}, shared_reservation);

  ASSERT_EQ(
    core->handle_goal(
      goal_id(101U), descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  ASSERT_TRUE(shared_reservation->snapshot().reserved);

  core.reset();
  EXPECT_FALSE(shared_reservation->snapshot().reserved);
}

TEST_F(TransferSequenceActionCoreTest, DestructionRetainsReservationUntilWorkerJoins)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_destructor_join", {}, {}, shared_reservation);
  const auto id = goal_id(111U);
  auto probe = std::make_shared<test::TransportProbe>();
  std::atomic_bool backend_started{false};
  std::promise<void> release_backend;
  const auto release_future = release_backend.get_future().share();

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [&backend_started, release_future](TransferSequenceTaskContext &) {
        backend_started.store(true);
        release_future.wait();
        return successful_execution();
      }),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));
  ASSERT_TRUE(spin_until([&backend_started]() {return backend_started.load();}));
  remove_node_from_executor();

  std::promise<void> destruction_started;
  auto destruction_started_future = destruction_started.get_future();
  std::promise<void> destruction_finished;
  auto destruction_finished_future = destruction_finished.get_future();
  std::thread destroyer(
    [owned_core = std::move(core), &destruction_started,
    &destruction_finished]() mutable
    {
      destruction_started.set_value();
      owned_core.reset();
      destruction_finished.set_value();
    });

  const auto started_status = destruction_started_future.wait_for(1s);
  const auto premature_finish_status = destruction_finished_future.wait_for(50ms);
  const auto reservation_while_joining = shared_reservation->snapshot();
  release_backend.set_value();
  const auto finished_status = destruction_finished_future.wait_for(1s);
  destroyer.join();

  EXPECT_EQ(started_status, std::future_status::ready);
  EXPECT_EQ(premature_finish_status, std::future_status::timeout);
  EXPECT_TRUE(reservation_while_joining.reserved);
  EXPECT_EQ(finished_status, std::future_status::ready);
  EXPECT_FALSE(shared_reservation->snapshot().reserved);
}

TEST_F(TransferSequenceActionCoreTest, UnreliableTerminalRetainsThenReleasesSharedReservation)
{
  auto shared_reservation = std::make_shared<TransferSequenceGoalReservation>();
  auto core = make_core("shared_terminal", {}, {}, shared_reservation);
  const auto id = goal_id(121U);
  auto probe = std::make_shared<test::TransportProbe>();
  probe->state = SequenceGoalTransportState::kOther;

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));

  ASSERT_TRUE(spin_until([&core]() {return core->snapshot().completion_pending;}));
  EXPECT_TRUE(shared_reservation->snapshot().reserved);
  EXPECT_EQ(probe->delivery_success_count, 0U);

  probe->state = SequenceGoalTransportState::kExecuting;
  ASSERT_TRUE(spin_until([&probe]() {return probe->delivery_success_count == 1U;}));
  ASSERT_TRUE(spin_until([&shared_reservation]() {
      return !shared_reservation->snapshot().reserved;
    }));
  EXPECT_FALSE(core->snapshot().goal_reserved);
}

}  // namespace
}  // namespace manipulation::workflow::detail
