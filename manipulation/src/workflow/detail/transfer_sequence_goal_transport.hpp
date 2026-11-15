// Responsibility: Defines the action-neutral transport boundary for one sequence goal.

#ifndef WORKFLOW__DETAIL__TRANSFER_SEQUENCE_GOAL_TRANSPORT_HPP_
#define WORKFLOW__DETAIL__TRANSFER_SEQUENCE_GOAL_TRANSPORT_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>

#include "manipulation/workflow/transfer_sequence_action_types.hpp"

namespace manipulation::workflow::detail
{

/// Transport state relevant to terminal delivery of an accepted sequence goal.
enum class SequenceGoalTransportState : std::uint8_t
{
  kExecuting,
  kCanceling,
  kOther,
};

/// Terminal transition selected by the action-independent sequence lifecycle.
enum class SequenceTerminalKind : std::uint8_t
{
  kSucceeded,
  kCanceled,
  kAborted,
};

/// Binary goal identity shared by ROS action and pause-control boundaries.
using SequenceGoalId = std::array<std::uint8_t, 16U>;

/**
 * Goal-scoped output port owned by the sequence lifecycle.
 *
 * The port owns a copy of the goal ID and its three callable objects. It does
 * not own or expose a ROS action server, goal handle, lifecycle core, or
 * worker. In particular, callback closures must not capture the lifecycle by
 * owning pointer because that would create an ownership cycle. Reference
 * captures remain the caller's responsibility and must outlive this port.
 *
 * Calls are forwarded synchronously and exceptions propagate unchanged. The
 * lifecycle owner serializes access and restores terminal evidence after a
 * failed delivery; this value object deliberately carries no delivery state.
 */
class SequenceGoalTransportPort final
{
public:
  using StateCallback = std::function<SequenceGoalTransportState()>;
  using PublishFeedbackCallback =
    std::function<void(const TransferSequenceFeedbackSnapshot &)>;
  using DeliverCallback = std::function<void(
        SequenceTerminalKind,
        const TransferSequenceExecution &,
        std::chrono::steady_clock::duration)>;

  /**
   * Bind all operations required for one accepted goal.
   *
   * Throws std::invalid_argument when any callback is empty. The callbacks
   * are otherwise opaque and are invoked only by the matching forwarder.
   */
  SequenceGoalTransportPort(
    SequenceGoalId goal_id,
    StateCallback state_callback,
    PublishFeedbackCallback publish_feedback_callback,
    DeliverCallback deliver_callback);

  SequenceGoalTransportPort(const SequenceGoalTransportPort &) = delete;
  SequenceGoalTransportPort & operator=(const SequenceGoalTransportPort &) = delete;
  SequenceGoalTransportPort(SequenceGoalTransportPort &&) noexcept = default;
  SequenceGoalTransportPort & operator=(SequenceGoalTransportPort &&) noexcept = default;
  ~SequenceGoalTransportPort() = default;

  /// Return the immutable goal identity copied at construction.
  [[nodiscard]] const SequenceGoalId & goal_id() const noexcept;

  /// Query the current external transport state.
  [[nodiscard]] SequenceGoalTransportState state() const;

  /// Forward one validated feedback snapshot synchronously.
  void publish_feedback(const TransferSequenceFeedbackSnapshot & feedback) const;

  /// Forward one terminal transition with its complete evidence and elapsed time.
  void deliver(
    SequenceTerminalKind kind,
    const TransferSequenceExecution & execution,
    std::chrono::steady_clock::duration elapsed) const;

private:
  SequenceGoalId goal_id_;
  StateCallback state_callback_;
  PublishFeedbackCallback publish_feedback_callback_;
  DeliverCallback deliver_callback_;
};

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_SEQUENCE_GOAL_TRANSPORT_HPP_
