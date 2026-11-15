// Responsibility: Captures coherent neutral sequence lifecycle diagnostics.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <chrono>
#include <memory>

#include "detail/transfer_sequence_action_active_goal.hpp"

namespace manipulation::workflow::detail
{

TransferSequenceActionServerSnapshot
TransferSequenceActionLifecycleCore::snapshot() const
{
  TransferSequenceActionServerSnapshot result;
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    result.goal_reserved = goal_reserved_;
    active = active_goal_;
  }

  const auto worker = worker_.snapshot();
  result.worker_state = worker.state;
  if (!active) {
    return result;
  }

  const auto session = active->session.snapshot();
  result.active = session.phase != TransferSequenceSessionPhase::kDelivered;
  result.stop_cause = session.stop_cause;
  result.backend_invoked = session.backend_invoked;
  result.completion_pending = session.completion_ready || session.delivery_in_progress;
  {
    const std::lock_guard<std::mutex> lock(active->feedback_mutex);
    result.phase = active->feedback.current_state.phase;
    result.sequence_step_index = static_cast<std::uint32_t>(active->feedback.step_index);
    result.sequence_step_count = static_cast<std::uint32_t>(active->feedback.step_count);
    result.overall_progress = active->feedback.overall_progress;
    result.paused = active->feedback.current_state.paused;
    result.pause_pending = active->feedback.current_state.pause_pending;
  }

  const auto now = Clock::now();
  result.elapsed = now > active->started_at ? now - active->started_at :
    Clock::duration::zero();
  if (session.worker_started) {
    result.paused = result.paused || worker.execution_control.paused_at_checkpoint;
    result.pause_pending = !result.paused &&
      (result.pause_pending || session.pause_requested ||
      worker.execution_control.pause_requested);
  } else {
    result.pause_pending = session.pause_requested;
  }
  return result;
}

}  // namespace manipulation::workflow::detail
