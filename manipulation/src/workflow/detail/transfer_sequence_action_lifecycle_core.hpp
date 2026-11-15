// Responsibility: Owns the action-independent single-goal lifecycle for ordered transfers.

#ifndef WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_LIFECYCLE_CORE_HPP_
#define WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_LIFECYCLE_CORE_HPP_

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/single_goal_task_worker.hpp"
#include "manipulation/workflow/transfer_sequence_action_types.hpp"
#include "manipulation/workflow/transfer_sequence_goal_reservation.hpp"
#include "manipulation/workflow/transfer_sequence_task_context.hpp"
#include "manipulation_msgs/srv/set_task_paused.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "transfer_sequence_goal_transport.hpp"

namespace manipulation::workflow::detail
{

/** Validated action-wire facts needed by the neutral lifecycle core. */
struct TransferSequenceGoalDescriptor
{
  using Duration = std::chrono::steady_clock::duration;

  std::size_t step_count{0U};
  /// Empty selects the core's default timeout; otherwise the value is positive.
  std::optional<Duration> requested_timeout;
};

/**
 * Run one ordered-transfer action goal independently of its generated ROS type.
 *
 * A typed facade decodes immutable goal data into TransferSequenceGoalDescriptor,
 * binds admission/execution callbacks to that goal, and supplies a validated
 * SequenceGoalTransportPort after ROS accepts it. The core never reads a typed
 * goal or goal handle. The supplied node, callback group, and action transport
 * must outlive this object, and the executor must be stopped before destruction.
 * The non-null shared reservation serializes backend admission across typed
 * facades that operate the same physical task resource.
 */
class TransferSequenceActionLifecycleCore final
{
public:
  using Clock = std::chrono::steady_clock;
  using PauseService = manipulation_msgs::srv::SetTaskPaused;
  using AdmissionCallback = std::function<TransferSequenceAdmission()>;
  using ExecuteCallback =
    std::function<TransferSequenceExecution(TransferSequenceTaskContext &)>;

  TransferSequenceActionLifecycleCore(
    rclcpp::Node & node,
    TransferSequenceLifecycleConfig config,
    std::string action_label,
    std::string pause_service_name,
    TransferSequenceRequestStopCallback request_stop,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    std::shared_ptr<TransferSequenceGoalReservation> goal_reservation);
  ~TransferSequenceActionLifecycleCore();

  TransferSequenceActionLifecycleCore(
    const TransferSequenceActionLifecycleCore &) = delete;
  TransferSequenceActionLifecycleCore & operator=(
    const TransferSequenceActionLifecycleCore &) = delete;
  TransferSequenceActionLifecycleCore(
    TransferSequenceActionLifecycleCore &&) = delete;
  TransferSequenceActionLifecycleCore & operator=(
    TransferSequenceActionLifecycleCore &&) = delete;

  /**
   * Admit and reserve one typed-facade goal without retaining its wire type.
   *
   * A missing descriptor represents malformed wire fields. The execute
   * callback is retained only after admission and reservation succeed.
   */
  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(
    const SequenceGoalId & goal_id,
    std::optional<TransferSequenceGoalDescriptor> descriptor,
    AdmissionCallback admission,
    ExecuteCallback execute);

  /// Record an exact cancel intent for the reserved goal UUID.
  [[nodiscard]] rclcpp_action::CancelResponse handle_cancel(
    const SequenceGoalId & goal_id);

  /// Bind the accepted ROS transport to its earlier reservation.
  void handle_accepted(SequenceGoalTransportPort transport);

  /// Capture lifecycle and sequence progress without exposing transport state.
  [[nodiscard]] TransferSequenceActionServerSnapshot snapshot() const;

private:
  struct ActiveGoal;

  void handle_pause_request(
    const std::shared_ptr<PauseService::Request> request,
    std::shared_ptr<PauseService::Response> response);

  void start_reserved_goal_if_ready();
  [[nodiscard]] manipulation::task::TaskOutcome execute_backend(
    const std::shared_ptr<ActiveGoal> & active,
    manipulation::task::ExecutionControl & control);
  void queue_completion(
    const std::shared_ptr<ActiveGoal> & active,
    manipulation::task::TaskOutcome outcome);
  [[nodiscard]] bool completion_is_queued(
    const std::shared_ptr<ActiveGoal> & active) const;
  void complete_goal(
    const std::shared_ptr<ActiveGoal> & active,
    manipulation::task::TaskOutcome outcome);
  void heartbeat();

  void accept_feedback(
    const std::shared_ptr<ActiveGoal> & active,
    TransferSequenceFeedbackSnapshot feedback);
  void publish_feedback(const std::shared_ptr<ActiveGoal> & active);
  void publish_terminal_feedback(
    const std::shared_ptr<ActiveGoal> & active,
    const manipulation::task::TaskOutcome & outcome);
  [[nodiscard]] TaskStopCause stop_cause(
    const std::shared_ptr<ActiveGoal> & active) const;
  [[nodiscard]] bool set_stop_cause_if_none(
    const std::shared_ptr<ActiveGoal> & active,
    TaskStopCause cause);
  void dispatch_backend_stop(const std::shared_ptr<ActiveGoal> & active);

  [[nodiscard]] bool reserve_goal(
    const SequenceGoalId & goal_id,
    manipulation::task::ObjectDisposition initial_disposition,
    Clock::time_point started_at,
    Clock::duration timeout,
    Clock::time_point deadline,
    std::size_t step_count,
    TransferSequenceGoalReservation::Lease goal_reservation_lease,
    ExecuteCallback execute);
  void release_completed_reservation();
  [[nodiscard]] bool request_matches_active_goal(
    const unique_identifier_msgs::msg::UUID & requested_goal_id,
    std::shared_ptr<ActiveGoal> & active) const;
  [[nodiscard]] bool is_current_goal(
    const std::shared_ptr<ActiveGoal> & active) const;

  const TransferSequenceLifecycleConfig config_;
  const std::string action_label_;
  const TransferSequenceRequestStopCallback request_stop_;
  const rclcpp::Logger logger_;
  const std::shared_ptr<TransferSequenceGoalReservation> goal_reservation_;

  mutable std::mutex goal_mutex_;
  bool goal_reserved_{false};
  bool terminal_delivered_{false};
  bool worker_start_pending_{false};
  bool worker_start_claimed_{false};
  bool reservation_cancel_pending_{false};
  SequenceGoalId reserved_goal_id_{};
  manipulation::task::ObjectDisposition reserved_initial_disposition_{
    manipulation::task::ObjectDisposition::kUnknown};
  Clock::time_point reserved_started_at_{};
  Clock::duration reserved_timeout_{};
  Clock::time_point reserved_deadline_{};
  std::size_t reserved_step_count_{0U};
  std::optional<TransferSequenceGoalReservation::Lease> reserved_goal_lease_;
  ExecuteCallback reserved_execute_;
  std::shared_ptr<const SequenceGoalTransportPort> pending_transport_;
  Clock::time_point worker_start_not_before_{};
  std::shared_ptr<ActiveGoal> active_goal_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<PauseService>::SharedPtr pause_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Declared last so destruction joins work before the shared lease, goal state,
  // and transport ports die.
  SingleGoalTaskWorker worker_;
};

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_LIFECYCLE_CORE_HPP_
