// Responsibility: Handles neutral sequence admission, cancel, acceptance, and pause requests.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "detail/transfer_sequence_action_active_goal.hpp"
#include "detail/transfer_sequence_action_time.hpp"

namespace manipulation::workflow::detail
{
namespace
{

bool valid_descriptor(const TransferSequenceGoalDescriptor & descriptor)
{
  return descriptor.step_count > 0U &&
         descriptor.step_count <= std::numeric_limits<std::uint32_t>::max() &&
         (!descriptor.requested_timeout ||
         *descriptor.requested_timeout > std::chrono::steady_clock::duration::zero());
}

TransferSequenceExecution unowned_reservation_failure()
{
  const auto outcome = manipulation::task::TaskOutcome::make_failure(
    manipulation::task::TaskErrorCode::kInternalError,
    manipulation::task::ObjectDisposition::kUnknown,
    0U, "accepted sequence goal does not own the reserved task slot");
  return {TransferSequenceResult{outcome, 0U, std::nullopt, {}}, {}};
}

}  // namespace

rclcpp_action::GoalResponse TransferSequenceActionLifecycleCore::handle_goal(
  const SequenceGoalId & goal_id,
  std::optional<TransferSequenceGoalDescriptor> descriptor,
  AdmissionCallback admission_callback,
  ExecuteCallback execute_callback)
{
  const auto received_at = Clock::now();
  release_completed_reservation();
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_reserved_) {
      RCLCPP_WARN(
        logger_, "Rejecting concurrent %s goal: task slot is reserved",
        action_label_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (!descriptor || !valid_descriptor(*descriptor)) {
    RCLCPP_WARN(
      logger_, "Rejecting %s goal: wire fields are malformed", action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!admission_callback || !execute_callback) {
    RCLCPP_ERROR(
      logger_, "%s goal callbacks are incomplete", action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto timeout = descriptor->requested_timeout.value_or(config_.default_timeout);
  const auto requested_deadline = sequence_saturating_deadline(received_at, timeout);

  std::optional<TransferSequenceGoalReservation::Lease> shared_lease;
  try {
    shared_lease = goal_reservation_->try_acquire(action_label_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "%s shared reservation failed: %s", action_label_.c_str(), error.what());
    return rclcpp_action::GoalResponse::REJECT;
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "%s shared reservation failed: unknown exception", action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!shared_lease) {
    RCLCPP_WARN(
      logger_, "Rejecting concurrent %s goal: shared task slot is reserved",
      action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  TransferSequenceAdmission admission;
  try {
    admission = admission_callback();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      logger_, "%s admission callback failed: %s", action_label_.c_str(), error.what());
    return rclcpp_action::GoalResponse::REJECT;
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "%s admission callback failed: unknown exception", action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!admission.accepted) {
    RCLCPP_WARN(
      logger_, "Rejecting %s goal in backend admission: %s",
      action_label_.c_str(),
      admission.reason.empty() ? "backend rejected the sequence goal" :
      admission.reason.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (Clock::now() >= requested_deadline) {
    RCLCPP_WARN(
      logger_, "Rejecting %s goal: admission exhausted its deadline",
      action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!reserve_goal(
      goal_id, admission.initial_disposition, received_at, timeout,
      requested_deadline, descriptor->step_count, std::move(*shared_lease),
      std::move(execute_callback)))
  {
    RCLCPP_WARN(
      logger_, "Rejecting concurrent %s goal: task slot is reserved",
      action_label_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TransferSequenceActionLifecycleCore::handle_cancel(
  const SequenceGoalId & goal_id)
{
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_reserved_ || goal_id != reserved_goal_id_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    active = active_goal_;
    if (!active) {
      reservation_cancel_pending_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }

  // The session linearizes an accepted cancel against backend completion.
  return active->session.request_stop(TaskStopCause::kClientCanceled) ?
         rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
}

void TransferSequenceActionLifecycleCore::handle_accepted(
  SequenceGoalTransportPort transport)
{
  const auto accepted_at = Clock::now();
  auto goal_transport =
    std::make_shared<const SequenceGoalTransportPort>(std::move(transport));
  auto active = std::make_shared<ActiveGoal>(goal_transport);
  active->goal_id = goal_transport->goal_id();

  bool owns_reservation = false;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    owns_reservation = goal_reserved_ && active->goal_id == reserved_goal_id_;
    if (owns_reservation) {
      if (reservation_cancel_pending_) {
        (void)active->session.request_stop(TaskStopCause::kClientCanceled);
      }
      active->execute = std::move(reserved_execute_);
      active->initial_disposition = reserved_initial_disposition_;
      active->started_at = reserved_started_at_;
      active->timeout = reserved_timeout_;
      active->deadline = reserved_deadline_;
      active->step_count = reserved_step_count_;
      active->feedback.step_index = 0U;
      active->feedback.step_count = active->step_count;
      active->feedback.current_state.phase = manipulation::task::TaskPhase::kValidating;
      active->feedback.current_state.detail = action_label_ + " goal accepted";
      active_goal_ = active;
      pending_transport_ = goal_transport;
      const auto requested_start = sequence_saturating_deadline(
        accepted_at, config_.accepted_start_delay);
      worker_start_not_before_ = std::min(requested_start, active->deadline);
      worker_start_pending_ = true;
      worker_start_claimed_ = false;
    }
  }

  if (!owns_reservation) {
    try {
      const auto execution = unowned_reservation_failure();
      goal_transport->deliver(
        SequenceTerminalKind::kAborted, execution, Clock::duration::zero());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        logger_, "Could not deliver unowned %s result: %s",
        action_label_.c_str(), error.what());
    } catch (...) {
      RCLCPP_ERROR(
        logger_, "Could not deliver unowned %s result: unknown exception",
        action_label_.c_str());
    }
    return;
  }

  publish_feedback(active);
  if (config_.accepted_start_delay == Clock::duration::zero()) {
    start_reserved_goal_if_ready();
  }
}

void TransferSequenceActionLifecycleCore::handle_pause_request(
  const std::shared_ptr<PauseService::Request> request,
  std::shared_ptr<PauseService::Response> response)
{
  std::shared_ptr<ActiveGoal> active;
  if (!request_matches_active_goal(request->goal_id, active)) {
    response->message = "no matching active sequence goal";
    return;
  }

  bool accepted = false;
  const auto session = active->session.snapshot();
  if (!session.worker_started) {
    accepted = active->session.set_pause_requested(request->paused);
  } else {
    accepted = request->paused ? worker_.request_pause() : worker_.resume();
    if (accepted) {
      (void)active->session.set_pause_requested(request->paused);
    }
  }

  response->accepted = accepted;
  response->message = accepted ?
    (request->paused ? "sequence pause requested" : "sequence resume requested") :
    "active sequence goal is already completing";
  if (accepted) {
    publish_feedback(active);
  }
}

}  // namespace manipulation::workflow::detail
