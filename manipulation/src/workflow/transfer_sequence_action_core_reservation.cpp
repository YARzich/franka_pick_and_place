// Responsibility: Owns neutral sequence reservation, reuse, and pause UUID matching.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "detail/transfer_sequence_action_active_goal.hpp"

namespace manipulation::workflow::detail
{
namespace
{

bool is_all_zero(const unique_identifier_msgs::msg::UUID & goal_id)
{
  return std::all_of(
    goal_id.uuid.cbegin(), goal_id.uuid.cend(),
    [](const std::uint8_t value) {return value == 0U;});
}

}  // namespace

bool TransferSequenceActionLifecycleCore::reserve_goal(
  const SequenceGoalId & goal_id,
  const manipulation::task::ObjectDisposition initial_disposition,
  const Clock::time_point started_at,
  const Clock::duration timeout,
  const Clock::time_point deadline,
  const std::size_t step_count,
  TransferSequenceGoalReservation::Lease goal_reservation_lease,
  ExecuteCallback execute)
{
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (goal_reserved_ || reserved_goal_lease_) {
    return false;
  }
  goal_reserved_ = true;
  terminal_delivered_ = false;
  worker_start_pending_ = false;
  worker_start_claimed_ = false;
  reservation_cancel_pending_ = false;
  reserved_goal_id_ = goal_id;
  reserved_initial_disposition_ = initial_disposition;
  reserved_started_at_ = started_at;
  reserved_timeout_ = timeout;
  reserved_deadline_ = deadline;
  reserved_step_count_ = step_count;
  reserved_goal_lease_.emplace(std::move(goal_reservation_lease));
  reserved_execute_ = std::move(execute);
  pending_transport_.reset();
  active_goal_.reset();
  return true;
}

void TransferSequenceActionLifecycleCore::release_completed_reservation()
{
  std::optional<TransferSequenceGoalReservation::Lease> released_shared_lease;
  std::shared_ptr<ActiveGoal> active;
  bool delivered = false;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
    delivered = goal_reserved_ && terminal_delivered_;
  }
  if (!delivered) {
    return;
  }

  const auto worker_state = worker_.snapshot().state;
  const bool worker_reusable = !active || !active->session.snapshot().worker_started ||
    worker_state == TaskWorkerState::kIdle || worker_state == TaskWorkerState::kCompleted;
  if (!worker_reusable) {
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_reserved_ || !terminal_delivered_ || active_goal_ != active) {
      return;
    }
    goal_reserved_ = false;
    terminal_delivered_ = false;
    worker_start_pending_ = false;
    worker_start_claimed_ = false;
    reservation_cancel_pending_ = false;
    reserved_goal_id_.fill(0U);
    reserved_initial_disposition_ = manipulation::task::ObjectDisposition::kUnknown;
    reserved_started_at_ = {};
    reserved_timeout_ = {};
    reserved_deadline_ = {};
    reserved_step_count_ = 0U;
    if (reserved_goal_lease_) {
      released_shared_lease.emplace(std::move(*reserved_goal_lease_));
      reserved_goal_lease_.reset();
    }
    reserved_execute_ = {};
    pending_transport_.reset();
    active_goal_.reset();
  }
  // Destroy the RAII lease after releasing goal_mutex_. Lease destruction
  // locks the shared reservation used by sibling action facades.
  released_shared_lease.reset();
}

bool TransferSequenceActionLifecycleCore::request_matches_active_goal(
  const unique_identifier_msgs::msg::UUID & requested_goal_id,
  std::shared_ptr<ActiveGoal> & active) const
{
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!goal_reserved_ || !active_goal_) {
    return false;
  }
  if (!is_all_zero(requested_goal_id) && requested_goal_id.uuid != reserved_goal_id_) {
    return false;
  }
  active = active_goal_;
  return true;
}

bool TransferSequenceActionLifecycleCore::is_current_goal(
  const std::shared_ptr<ActiveGoal> & active) const
{
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  return goal_reserved_ && active_goal_ == active &&
         active && active->goal_id == reserved_goal_id_;
}

}  // namespace manipulation::workflow::detail
