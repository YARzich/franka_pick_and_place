// Responsibility: Runs the admitted ServoToPose task and converts exceptional exits to results.

#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "servo_to_pose_active_goal.hpp"

namespace manipulation::moveit
{

void ServoToPoseActionServer::run_task(const std::shared_ptr<ActiveGoal> & active)
{
  ServoToPoseTaskResult result;
  try {
    result = task_.run(
      active->goal_handle->get_goal()->target_pose,
      active->started_at,
      active->deadline,
      *active->control,
      [this, active](const ServoToPoseTaskFeedback & feedback) {
        accept_feedback(active, feedback);
      });
  } catch (const std::exception & error) {
    result = task_.halt_after_unexpected_failure(
      active->started_at,
      active->deadline,
      std::string{"ServoToPose task escaped with exception: "} + error.what());
  } catch (...) {
    result = task_.halt_after_unexpected_failure(
      active->started_at,
      active->deadline,
      "ServoToPose task escaped with an unknown exception");
  }

  const std::lock_guard<std::mutex> lock(active->mutex);
  if (result.succeeded() && active->control->snapshot().cancel_requested) {
    // The cancel callback takes this same mutex before setting the intent. It
    // therefore either wins here and produces a coherent canceled payload, or
    // observes worker_finished below and rejects the late cancellation.
    result.error_code = manipulation::task::TaskErrorCode::kCanceled;
    result.detail = "ServoToPose canceled before terminal result commit";
  }
  active->result = std::move(result);
  active->worker_running = false;
  active->worker_finished = true;
}

void ServoToPoseActionServer::accept_feedback(
  const std::shared_ptr<ActiveGoal> & active,
  const ServoToPoseTaskFeedback & feedback)
{
  std::optional<manipulation::recording::EpisodeLifecycleMarker> marker;
  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    const bool phase_changed = feedback.state.phase != active->last_recorded_phase;
    const bool pause_changed = feedback.state.paused != active->last_recorded_paused;
    active->feedback = feedback;
    if (pause_changed) {
      marker = feedback.state.paused ?
        manipulation::recording::EpisodeLifecycleMarker::kPaused :
        manipulation::recording::EpisodeLifecycleMarker::kResumed;
    } else if (phase_changed) {
      marker = manipulation::recording::EpisodeLifecycleMarker::kStateChanged;
    }
    active->last_recorded_phase = feedback.state.phase;
    active->last_recorded_paused = feedback.state.paused;
  }
  if (marker) {
    record_event(active, *marker);
  }
}

}  // namespace manipulation::moveit
