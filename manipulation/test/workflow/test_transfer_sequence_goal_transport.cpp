// Responsibility: Verifies the goal transport port's validation and exact forwarding contract.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "gtest/gtest.h"

#include "detail/transfer_sequence_goal_transport.hpp"
#include "manipulation/task/task_status.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using namespace std::chrono_literals;
using manipulation::task::ObjectDisposition;
using manipulation::task::TaskErrorCode;
using manipulation::task::TaskOutcome;
using manipulation::task::TaskPhase;

static_assert(!std::is_copy_constructible_v<SequenceGoalTransportPort>);
static_assert(!std::is_copy_assignable_v<SequenceGoalTransportPort>);
static_assert(std::is_nothrow_move_constructible_v<SequenceGoalTransportPort>);
static_assert(std::is_nothrow_move_assignable_v<SequenceGoalTransportPort>);

SequenceGoalId sample_goal_id()
{
  SequenceGoalId goal_id{};
  for (std::size_t index = 0U; index < goal_id.size(); ++index) {
    goal_id[index] = static_cast<std::uint8_t>(index + 1U);
  }
  return goal_id;
}

SequenceGoalTransportPort::StateCallback valid_state_callback()
{
  return []() {return SequenceGoalTransportState::kExecuting;};
}

SequenceGoalTransportPort::PublishFeedbackCallback valid_feedback_callback()
{
  return [](const TransferSequenceFeedbackSnapshot &) {};
}

SequenceGoalTransportPort::DeliverCallback valid_deliver_callback()
{
  return [](
    SequenceTerminalKind,
    const TransferSequenceExecution &,
    std::chrono::steady_clock::duration) {};
}

TEST(SequenceGoalTransportPort, RejectsEveryMissingCallback)
{
  EXPECT_THROW(
    SequenceGoalTransportPort(
      sample_goal_id(), {}, valid_feedback_callback(), valid_deliver_callback()),
    std::invalid_argument);
  EXPECT_THROW(
    SequenceGoalTransportPort(
      sample_goal_id(), valid_state_callback(), {}, valid_deliver_callback()),
    std::invalid_argument);
  EXPECT_THROW(
    SequenceGoalTransportPort(
      sample_goal_id(), valid_state_callback(), valid_feedback_callback(), {}),
    std::invalid_argument);
}

TEST(SequenceGoalTransportPort, ForwardsIdentityStateFeedbackAndTerminalEvidenceExactly)
{
  const auto goal_id = sample_goal_id();
  auto external_state = SequenceGoalTransportState::kExecuting;
  const TransferSequenceFeedbackSnapshot * published_feedback = nullptr;
  const TransferSequenceExecution * delivered_execution = nullptr;
  auto delivered_kind = SequenceTerminalKind::kAborted;
  auto delivered_elapsed = std::chrono::steady_clock::duration::zero();
  std::size_t state_call_count = 0U;
  std::size_t feedback_call_count = 0U;
  std::size_t deliver_call_count = 0U;

  SequenceGoalTransportPort transport(
    goal_id,
    [&external_state, &state_call_count]() {
      ++state_call_count;
      return external_state;
    },
    [&published_feedback, &feedback_call_count](
      const TransferSequenceFeedbackSnapshot & feedback) {
      ++feedback_call_count;
      published_feedback = &feedback;
    },
    [&delivered_execution, &delivered_kind, &delivered_elapsed, &deliver_call_count](
      const SequenceTerminalKind kind,
      const TransferSequenceExecution & execution,
      const std::chrono::steady_clock::duration elapsed) {
      ++deliver_call_count;
      delivered_kind = kind;
      delivered_execution = &execution;
      delivered_elapsed = elapsed;
    });

  EXPECT_EQ(transport.goal_id(), goal_id);
  EXPECT_EQ(transport.state(), SequenceGoalTransportState::kExecuting);
  external_state = SequenceGoalTransportState::kCanceling;
  EXPECT_EQ(transport.state(), SequenceGoalTransportState::kCanceling);
  EXPECT_EQ(state_call_count, 2U);

  TransferSequenceFeedbackSnapshot feedback;
  feedback.step_index = 2U;
  feedback.step_count = 4U;
  feedback.overall_progress = 0.625F;
  feedback.current_state.phase = TaskPhase::kTransferring;
  feedback.current_state.progress = 0.5F;
  feedback.current_state.elapsed = 17ms;
  feedback.current_state.detail = "moving object";
  transport.publish_feedback(feedback);

  ASSERT_EQ(published_feedback, &feedback);
  EXPECT_EQ(feedback_call_count, 1U);
  EXPECT_EQ(published_feedback->step_index, 2U);
  EXPECT_EQ(published_feedback->step_count, 4U);
  EXPECT_FLOAT_EQ(published_feedback->overall_progress, 0.625F);
  EXPECT_EQ(published_feedback->current_state.phase, TaskPhase::kTransferring);
  EXPECT_EQ(published_feedback->current_state.elapsed, 17ms);
  EXPECT_EQ(published_feedback->current_state.detail, "moving object");

  const auto successful_step = TaskOutcome::make_success(
    ObjectDisposition::kObjectAtTarget, 1U, "placed after recovery");
  const auto failed_step = TaskOutcome::make_failure(
    TaskErrorCode::kPlanningFailed,
    ObjectDisposition::kObjectAtSource,
    2U,
    "second step could not be planned");
  const TransferSequenceExecution execution{
    TransferSequenceResult{failed_step, 1U, 1U, {successful_step, failed_step}},
    {31ms, 47ms}};
  transport.deliver(SequenceTerminalKind::kAborted, execution, 83ms);

  ASSERT_EQ(delivered_execution, &execution);
  EXPECT_EQ(deliver_call_count, 1U);
  EXPECT_EQ(delivered_kind, SequenceTerminalKind::kAborted);
  EXPECT_EQ(delivered_elapsed, 83ms);
  EXPECT_EQ(delivered_execution->result.completed_step_count, 1U);
  ASSERT_EQ(delivered_execution->result.failed_step_index, 1U);
  ASSERT_EQ(delivered_execution->result.step_outcomes.size(), 2U);
  EXPECT_TRUE(delivered_execution->result.step_outcomes.front().succeeded());
  EXPECT_EQ(
    delivered_execution->result.step_outcomes.back().error_code(),
    TaskErrorCode::kPlanningFailed);
  EXPECT_EQ(
    delivered_execution->result.step_outcomes.back().recovery_count(), 2U);
  ASSERT_EQ(delivered_execution->step_elapsed.size(), 2U);
  EXPECT_EQ(delivered_execution->step_elapsed.front(), 31ms);
  EXPECT_EQ(delivered_execution->step_elapsed.back(), 47ms);
}

class CallbackFailure final : public std::runtime_error
{
public:
  explicit CallbackFailure(const std::string & detail)
  : std::runtime_error(detail)
  {
  }
};

TEST(SequenceGoalTransportPort, PropagatesCallbackFailuresAndRemainsUsable)
{
  bool throw_from_state = true;
  bool throw_from_feedback = true;
  bool throw_from_deliver = true;
  std::size_t feedback_success_count = 0U;
  std::size_t deliver_success_count = 0U;

  SequenceGoalTransportPort transport(
    sample_goal_id(),
    [&throw_from_state]() {
      if (std::exchange(throw_from_state, false)) {
        throw CallbackFailure("state failed");
      }
      return SequenceGoalTransportState::kOther;
    },
    [&throw_from_feedback, &feedback_success_count](
      const TransferSequenceFeedbackSnapshot &) {
      if (std::exchange(throw_from_feedback, false)) {
        throw CallbackFailure("feedback failed");
      }
      ++feedback_success_count;
    },
    [&throw_from_deliver, &deliver_success_count](
      SequenceTerminalKind,
      const TransferSequenceExecution &,
      std::chrono::steady_clock::duration) {
      if (std::exchange(throw_from_deliver, false)) {
        throw CallbackFailure("deliver failed");
      }
      ++deliver_success_count;
    });

  EXPECT_THROW((void)transport.state(), CallbackFailure);
  EXPECT_EQ(transport.state(), SequenceGoalTransportState::kOther);

  const TransferSequenceFeedbackSnapshot feedback;
  EXPECT_THROW(transport.publish_feedback(feedback), CallbackFailure);
  EXPECT_NO_THROW(transport.publish_feedback(feedback));
  EXPECT_EQ(feedback_success_count, 1U);

  const auto outcome = TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget);
  const TransferSequenceExecution execution{
    TransferSequenceResult{outcome, 1U, std::nullopt, {outcome}}, {3ms}};
  EXPECT_THROW(
    transport.deliver(SequenceTerminalKind::kSucceeded, execution, 5ms),
    CallbackFailure);
  EXPECT_NO_THROW(
    transport.deliver(SequenceTerminalKind::kSucceeded, execution, 5ms));
  EXPECT_EQ(deliver_success_count, 1U);
}

}  // namespace
}  // namespace manipulation::workflow::detail
