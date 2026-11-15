// Responsibility: Validates and forwards one sequence goal's transport callbacks.

#include "transfer_sequence_goal_transport.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::workflow::detail
{

SequenceGoalTransportPort::SequenceGoalTransportPort(
  SequenceGoalId goal_id,
  StateCallback state_callback,
  PublishFeedbackCallback publish_feedback_callback,
  DeliverCallback deliver_callback)
: goal_id_(std::move(goal_id)),
  state_callback_(std::move(state_callback)),
  publish_feedback_callback_(std::move(publish_feedback_callback)),
  deliver_callback_(std::move(deliver_callback))
{
  if (!state_callback_) {
    throw std::invalid_argument("SequenceGoalTransportPort requires a state callback");
  }
  if (!publish_feedback_callback_) {
    throw std::invalid_argument(
            "SequenceGoalTransportPort requires a publish-feedback callback");
  }
  if (!deliver_callback_) {
    throw std::invalid_argument("SequenceGoalTransportPort requires a deliver callback");
  }
}

const SequenceGoalId & SequenceGoalTransportPort::goal_id() const noexcept
{
  return goal_id_;
}

SequenceGoalTransportState SequenceGoalTransportPort::state() const
{
  return state_callback_();
}

void SequenceGoalTransportPort::publish_feedback(
  const TransferSequenceFeedbackSnapshot & feedback) const
{
  publish_feedback_callback_(feedback);
}

void SequenceGoalTransportPort::deliver(
  const SequenceTerminalKind kind,
  const TransferSequenceExecution & execution,
  const std::chrono::steady_clock::duration elapsed) const
{
  deliver_callback_(kind, execution, elapsed);
}

}  // namespace manipulation::workflow::detail
