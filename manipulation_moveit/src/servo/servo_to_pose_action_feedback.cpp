// Responsibility: Publishes ServoToPose feedback and terminal ROS action results.

#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

#include <exception>
#include <memory>
#include <optional>
#include <utility>

#include "manipulation/workflow/task_message_adapter.hpp"
#include "servo_to_pose_active_goal.hpp"

namespace manipulation::moveit
{
namespace
{

manipulation_msgs::msg::MotionTaskResult to_motion_result(
  const ServoToPoseTaskResult & result)
{
  manipulation_msgs::msg::MotionTaskResult message;
  message.success = result.succeeded();
  message.error_code = manipulation::workflow::to_wire_error_code(result.error_code);
  message.error_message = result.detail;
  message.stop_confirmed = result.stop_confirmed;
  message.recovery_count = 0U;
  message.elapsed = manipulation::workflow::to_ros_duration(result.elapsed);
  return message;
}

}  // namespace

void ServoToPoseActionServer::heartbeat()
{
  if (!backend_ready_announced_ && port_.backend_ready()) {
    backend_ready_announced_ = true;
    RCLCPP_INFO(logger_, "MoveIt Servo backend is ready for ServoToPose goals");
  }

  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
  }
  if (!active) {
    release_completed_goal();
    return;
  }

  bool result_pending = false;
  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    result_pending = active->worker_finished && active->result.has_value() &&
      !active->terminal_delivered;
  }
  if (result_pending) {
    deliver_result(active);
  } else {
    publish_feedback(active);
  }
  release_completed_goal();
}

void ServoToPoseActionServer::publish_feedback(
  const std::shared_ptr<ActiveGoal> & active)
{
  const std::lock_guard<std::mutex> publish_lock(active->publish_mutex);
  if (!active->goal_handle->is_executing()) {
    return;
  }

  ServoToPoseTaskFeedback feedback;
  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    feedback = active->feedback;
  }
  feedback.state.elapsed = std::chrono::steady_clock::now() - active->started_at;

  auto message = std::make_shared<Action::Feedback>();
  message->state = manipulation::workflow::to_ros_message(feedback.state);
  message->tracking_error_valid = feedback.tracking_error_valid;
  message->linear_error_m = feedback.tracking_error_valid ? feedback.linear_error_m : 0.0;
  message->angular_error_rad = feedback.tracking_error_valid ? feedback.angular_error_rad : 0.0;
  try {
    active->goal_handle->publish_feedback(message);
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger_, "Could not publish ServoToPose feedback: %s", error.what());
  }
}

void ServoToPoseActionServer::deliver_result(
  const std::shared_ptr<ActiveGoal> & active)
{
  std::optional<ServoToPoseTaskResult> result;
  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    if (active->terminal_delivered || !active->worker_finished || !active->result) {
      return;
    }
    result = active->result;
  }

  const bool canceling = active->goal_handle->is_canceling();
  const bool executing = active->goal_handle->is_executing();
  if (!canceling && !executing) {
    return;
  }

  auto wire_result = std::make_shared<Action::Result>();
  wire_result->result = to_motion_result(*result);
  try {
    if (canceling) {
      // ROS actions can only leave CANCELING through the canceled transition.
      // The typed payload still reports RECOVERY_FAILED when safe halt failed.
      active->goal_handle->canceled(wire_result);
    } else if (result->succeeded()) {
      active->goal_handle->succeed(wire_result);
    } else {
      active->goal_handle->abort(wire_result);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "Could not deliver ServoToPose result: %s", error.what());
    return;
  }

  const auto marker = result->succeeded() ?
    manipulation::recording::EpisodeLifecycleMarker::kSucceeded :
    (result->error_code == manipulation::task::TaskErrorCode::kCanceled ?
    manipulation::recording::EpisodeLifecycleMarker::kCanceled :
    manipulation::recording::EpisodeLifecycleMarker::kFailed);
  record_event(active, marker, &*result);

  {
    // Match the goal_mutex_ -> ActiveGoal::mutex order used by cleanup. The
    // lease crosses result delivery and, without a stop proof, moves into the
    // process-lifetime quarantine before the completed goal can be released.
    const std::lock_guard<std::mutex> goal_lock(goal_mutex_);
    const std::lock_guard<std::mutex> active_lock(active->mutex);
    if (!result->stop_confirmed && active->resource_lease &&
      active->resource_lease->valid())
    {
      if (!quarantined_resource_lease_) {
        quarantined_resource_lease_.emplace(std::move(*active->resource_lease));
        RCLCPP_ERROR(
          logger_,
          "ServoToPose could not prove a safe stop; arm resource is quarantined "
          "until the motion host restarts");
      }
    }
    active->resource_lease.reset();
    active->terminal_delivered = true;
  }
}

void ServoToPoseActionServer::release_completed_goal()
{
  std::thread completed_worker;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!active_goal_) {
      if (!goal_reserved_ && worker_.joinable()) {
        completed_worker = std::move(worker_);
      }
    } else {
      const std::lock_guard<std::mutex> active_lock(active_goal_->mutex);
      if (!active_goal_->terminal_delivered || !active_goal_->worker_finished) {
        return;
      }
      active_goal_.reset();
      goal_reserved_ = false;
      cancel_before_accept_ = false;
      reserved_goal_id_ = {};
      if (worker_.joinable()) {
        completed_worker = std::move(worker_);
      }
    }
  }
  if (completed_worker.joinable()) {
    completed_worker.join();
  }
}

ServoToPoseActionServerSnapshot ServoToPoseActionServer::snapshot() const
{
  ServoToPoseActionServerSnapshot snapshot;
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    snapshot.goal_reserved = goal_reserved_;
    snapshot.resource_quarantined = quarantined_resource_lease_.has_value() &&
      quarantined_resource_lease_->valid();
    snapshot.resource_held = snapshot.resource_quarantined;
    active = active_goal_;
  }
  snapshot.active = static_cast<bool>(active);
  if (!active) {
    return snapshot;
  }

  const std::lock_guard<std::mutex> lock(active->mutex);
  snapshot.worker_running = active->worker_running;
  snapshot.result_pending = active->result.has_value() && !active->terminal_delivered;
  snapshot.resource_held = snapshot.resource_held ||
    (active->resource_lease.has_value() && active->resource_lease->valid());
  snapshot.phase = active->feedback.state.phase;
  snapshot.paused = active->feedback.state.paused;
  snapshot.pause_pending = active->feedback.state.pause_pending;
  return snapshot;
}

}  // namespace manipulation::moveit
