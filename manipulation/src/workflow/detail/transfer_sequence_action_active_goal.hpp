// Responsibility: Collects mutable state shared by callbacks for one neutral sequence goal.

#ifndef WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_ACTIVE_GOAL_HPP_
#define WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_ACTIVE_GOAL_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "manipulation/workflow/detail/transfer_sequence_session.hpp"

#include "transfer_sequence_action_lifecycle_core.hpp"

namespace manipulation::workflow::detail
{

struct TransferSequenceActionLifecycleCore::ActiveGoal
{
  ActiveGoal(
    std::shared_ptr<const SequenceGoalTransportPort> goal_transport)
  : transport(std::move(goal_transport))
  {
  }

  std::shared_ptr<const SequenceGoalTransportPort> transport;
  ExecuteCallback execute;
  SequenceGoalId goal_id{};
  Clock::time_point started_at;
  Clock::duration timeout{};
  Clock::time_point deadline;
  std::size_t step_count{0U};
  manipulation::task::ObjectDisposition initial_disposition{
    manipulation::task::ObjectDisposition::kUnknown};

  mutable std::mutex feedback_mutex;
  mutable std::mutex publish_mutex;
  TransferSequenceFeedbackSnapshot feedback;

  // The worker API returns TaskOutcome, so richer sequence evidence travels
  // beside it and is consumed only during serialized terminal delivery.
  mutable std::mutex execution_mutex;
  std::optional<TransferSequenceExecution> backend_execution;

  TransferSequenceSession session;
  std::atomic_bool cancel_propagated{false};
};

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_ACTIVE_GOAL_HPP_
