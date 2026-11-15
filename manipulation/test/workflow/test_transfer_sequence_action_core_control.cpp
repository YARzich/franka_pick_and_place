// Responsibility: Verifies neutral core transport-state gating, retries, and reuse.

#include <atomic>
#include <chrono>
#include <memory>
#include <utility>

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

TEST_F(TransferSequenceActionCoreTest, RestoresCompletionUntilTransportIsExecuting)
{
  auto core = make_core("delayed_executing");
  const auto id = goal_id(1U);
  auto probe = std::make_shared<test::TransportProbe>();
  probe->state = SequenceGoalTransportState::kOther;

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));

  ASSERT_TRUE(spin_until([&core]() {return core->snapshot().completion_pending;}));
  EXPECT_EQ(probe->delivery_attempt_count, 0U);
  probe->state = SequenceGoalTransportState::kExecuting;
  ASSERT_TRUE(spin_until([&probe]() {return probe->delivery_success_count == 1U;}));

  EXPECT_EQ(probe->delivered_kind, SequenceTerminalKind::kSucceeded);
  ASSERT_TRUE(probe->delivered_execution.has_value());
  EXPECT_TRUE(probe->delivered_execution->result.outcome.succeeded());
}

TEST_F(TransferSequenceActionCoreTest, RestoresCanceledCompletionUntilCanceling)
{
  std::atomic_bool backend_started{false};
  std::atomic_bool release_backend{false};
  auto core = make_core(
    "delayed_canceling", {},
    [&release_backend](TaskStopCause) {release_backend.store(true);});
  const auto id = goal_id(21U);
  auto probe = std::make_shared<test::TransportProbe>();

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [&backend_started, &release_backend](TransferSequenceTaskContext &) {
        backend_started.store(true);
        while (!release_backend.load()) {
          std::this_thread::sleep_for(1ms);
        }
        return successful_execution();
      }),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));
  ASSERT_TRUE(spin_until([&backend_started]() {return backend_started.load();}));
  ASSERT_EQ(core->handle_cancel(id), rclcpp_action::CancelResponse::ACCEPT);

  ASSERT_TRUE(spin_until([&core]() {return core->snapshot().completion_pending;}));
  EXPECT_EQ(probe->delivery_attempt_count, 0U);
  probe->state = SequenceGoalTransportState::kCanceling;
  ASSERT_TRUE(spin_until([&probe]() {return probe->delivery_success_count == 1U;}));

  EXPECT_EQ(probe->delivered_kind, SequenceTerminalKind::kCanceled);
  ASSERT_TRUE(probe->delivered_execution.has_value());
  EXPECT_EQ(
    probe->delivered_execution->result.outcome.error_code(),
    manipulation::task::TaskErrorCode::kCanceled);
}

TEST_F(TransferSequenceActionCoreTest, UnknownTransportFailuresAreRetriedWithoutEvidenceLoss)
{
  auto core = make_core("transport_retry");
  const auto id = goal_id(41U);
  auto probe = std::make_shared<test::TransportProbe>();
  probe->throw_feedback_once = true;
  probe->throw_delivery_once = true;

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  EXPECT_NO_THROW(core->handle_accepted(transport_port(id, probe)));
  ASSERT_TRUE(spin_until([&probe]() {return probe->delivery_success_count == 1U;}));

  EXPECT_EQ(probe->delivery_attempt_count, 2U);
  ASSERT_TRUE(probe->delivered_execution.has_value());
  EXPECT_EQ(probe->delivered_execution->result.completed_step_count, 1U);
  ASSERT_EQ(probe->delivered_execution->result.step_outcomes.size(), 1U);
  EXPECT_TRUE(probe->delivered_execution->result.step_outcomes.front().succeeded());
  ASSERT_EQ(probe->delivered_execution->step_elapsed.size(), 1U);
  EXPECT_EQ(probe->delivered_execution->step_elapsed.front(), 1ms);
}

TEST_F(TransferSequenceActionCoreTest, UnknownStateFailureIsRetriedWithoutEvidenceLoss)
{
  auto core = make_core("state_retry");
  const auto id = goal_id(51U);
  auto probe = std::make_shared<test::TransportProbe>();
  probe->state = SequenceGoalTransportState::kOther;

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));

  ASSERT_TRUE(spin_until([&core]() {return core->snapshot().completion_pending;}));
  probe->throw_state_once = true;
  probe->state = SequenceGoalTransportState::kExecuting;
  ASSERT_TRUE(spin_until([&probe]() {return probe->delivery_success_count == 1U;}));

  EXPECT_EQ(probe->state_failure_count, 1U);
  EXPECT_EQ(probe->delivery_attempt_count, 1U);
  ASSERT_TRUE(probe->delivered_execution.has_value());
  EXPECT_TRUE(probe->delivered_execution->result.outcome.succeeded());
  EXPECT_EQ(probe->delivered_execution->result.completed_step_count, 1U);
}

TEST_F(TransferSequenceActionCoreTest, ReusesSlotWithoutCallingCompletedGoalTransport)
{
  auto core = make_core("reuse");
  const auto first_id = goal_id(61U);
  auto first_probe = std::make_shared<test::TransportProbe>();
  ASSERT_EQ(
    core->handle_goal(
      first_id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(first_id, first_probe));
  ASSERT_TRUE(spin_until([&core]() {return !core->snapshot().goal_reserved;}));
  ASSERT_EQ(first_probe->delivery_success_count, 1U);

  const auto second_id = goal_id(81U);
  auto second_probe = std::make_shared<test::TransportProbe>();
  ASSERT_EQ(
    core->handle_goal(
      second_id, descriptor(), accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution();}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(second_id, second_probe));
  ASSERT_TRUE(spin_until([&second_probe]() {
      return second_probe->delivery_success_count == 1U;
    }));

  EXPECT_EQ(first_probe->delivery_success_count, 1U);
  EXPECT_EQ(second_probe->delivery_success_count, 1U);
}

}  // namespace
}  // namespace manipulation::workflow::detail
