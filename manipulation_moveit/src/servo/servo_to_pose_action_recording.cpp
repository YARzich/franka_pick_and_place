// Responsibility: Emits best-effort lifecycle records for ServoToPose executions.

#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

#include <optional>

#include "servo_to_pose_active_goal.hpp"

namespace manipulation::moveit
{

void ServoToPoseActionServer::record_event(
  const std::shared_ptr<ActiveGoal> & active,
  const manipulation::recording::EpisodeLifecycleMarker marker,
  const ServoToPoseTaskResult * result) const noexcept
{
  try {
    manipulation::task::TaskStateSnapshot state;
    {
      const std::lock_guard<std::mutex> lock(active->mutex);
      state = active->feedback.state;
    }
    state.elapsed = std::chrono::steady_clock::now() - active->started_at;

    std::optional<manipulation::task::TaskOutcome> outcome;
    if (result != nullptr) {
      const auto disposition = result->stop_confirmed ?
        manipulation::task::ObjectDisposition::kSafeIdle :
        manipulation::task::ObjectDisposition::kUnknown;
      outcome = result->succeeded() ?
        manipulation::task::TaskOutcome::make_success(
        disposition, result->detail) :
        manipulation::task::TaskOutcome::make_failure(
        result->error_code,
        disposition,
        0U,
        result->detail);
    }
    (void)config_.skill_runtime->try_record(
      marker, active->episode_id, state.elapsed, state, outcome);
  } catch (...) {
    // Episode recording is best effort and cannot alter motion lifecycle.
  }
}

}  // namespace manipulation::moveit
