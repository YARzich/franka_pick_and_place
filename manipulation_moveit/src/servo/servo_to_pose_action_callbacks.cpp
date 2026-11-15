// Responsibility: Handles ServoToPose goal admission, cancellation, and pause intent callbacks.

#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "manipulation/validation/rigid_pose.hpp"
#include "manipulation/workflow/task_message_adapter.hpp"
#include "servo_to_pose_active_goal.hpp"

namespace manipulation::moveit
{
namespace
{

using Clock = std::chrono::steady_clock;
using ErrorCode = manipulation::task::TaskErrorCode;

bool valid_wire_duration(const builtin_interfaces::msg::Duration & duration) noexcept
{
  return duration.sec >= 0 && duration.nanosec < 1000000000U;
}

Clock::duration requested_timeout(
  const manipulation_msgs::action::ServoToPose::Goal & goal,
  const ServoToPoseActionServerConfig & config)
{
  if (goal.timeout.sec == 0 && goal.timeout.nanosec == 0U) {
    return config.default_timeout;
  }
  return std::chrono::duration_cast<Clock::duration>(
    std::chrono::seconds{goal.timeout.sec} +
    std::chrono::nanoseconds{goal.timeout.nanosec});
}

Clock::time_point saturating_deadline(
  const Clock::time_point start, const Clock::duration timeout) noexcept
{
  const auto maximum = Clock::time_point::max() - start;
  return timeout >= maximum ? Clock::time_point::max() : start + timeout;
}

std::string encode_goal_id(const rclcpp_action::GoalUUID & goal_id)
{
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0');
  for (const std::uint8_t byte : goal_id) {
    encoded << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return encoded.str();
}

std::string resource_owner_id(const rclcpp_action::GoalUUID & goal_id)
{
  return "goal/id_" + encode_goal_id(goal_id);
}

ServoToPoseTaskResult admission_failure(
  const ErrorCode code, std::string detail, const Clock::time_point started_at)
{
  return {code, false, Clock::now() - started_at, std::move(detail)};
}

}  // namespace

rclcpp_action::GoalResponse ServoToPoseActionServer::handle_goal(
  const rclcpp_action::GoalUUID & goal_id,
  const std::shared_ptr<const Action::Goal> goal)
{
  if (!port_.backend_ready()) {
    RCLCPP_WARN(logger_, "Rejecting ServoToPose goal: MoveIt Servo backend is not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!manipulation::validation::is_valid_pose_stamped(goal->target_pose) ||
    !valid_wire_duration(goal->timeout))
  {
    RCLCPP_WARN(logger_, "Rejecting invalid ServoToPose pose or timeout");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!servo_to_pose_timeout_has_operational_budget(requested_timeout(*goal, config_),
      config_.task))
  {
    RCLCPP_WARN(
      logger_,
      "Rejecting ServoToPose timeout without tracking budget before bounded halt reserve");
    return rclcpp_action::GoalResponse::REJECT;
  }

  release_completed_goal();
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (goal_reserved_) {
    RCLCPP_WARN(logger_, "Rejecting concurrent ServoToPose goal: task slot is reserved");
    return rclcpp_action::GoalResponse::REJECT;
  }
  goal_reserved_ = true;
  cancel_before_accept_ = false;
  reserved_goal_id_ = goal_id;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ServoToPoseActionServer::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_reserved_ || goal_handle->get_goal_id() != reserved_goal_id_) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    active = active_goal_;
    if (!active) {
      cancel_before_accept_ = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    const std::lock_guard<std::mutex> active_lock(active->mutex);
    if (active->worker_finished || active->terminal_delivered) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    // Linearize cancellation before the worker can commit its terminal result.
    active->control->request_cancel();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ServoToPoseActionServer::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto started_at = Clock::now();
  auto active = std::make_shared<ActiveGoal>();
  active->goal_handle = goal_handle;
  active->goal_id = goal_handle->get_goal_id();
  active->episode_id = encode_goal_id(active->goal_id);
  active->started_at = started_at;
  active->deadline = saturating_deadline(
    started_at, requested_timeout(*goal_handle->get_goal(), config_));
  active->control = std::make_shared<ServoToPoseTaskControl>();
  active->feedback.state.phase = manipulation::task::TaskPhase::kWaitingForResources;
  active->feedback.state.step_count = 1U;
  active->feedback.state.detail = "acquiring the shared arm control resource";

  bool owns_reservation = false;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    owns_reservation = goal_reserved_ && active->goal_id == reserved_goal_id_;
    if (owns_reservation) {
      if (cancel_before_accept_) {
        active->control->request_cancel();
      }
      active_goal_ = active;
    }
  }
  if (!owns_reservation) {
    const auto failure = admission_failure(
      ErrorCode::kInternalError,
      "accepted ServoToPose goal did not own its reserved slot", started_at);
    auto result = std::make_shared<Action::Result>();
    result->result.success = false;
    result->result.error_code = manipulation::workflow::to_wire_error_code(failure.error_code);
    result->result.error_message = failure.detail;
    result->result.stop_confirmed = false;
    result->result.elapsed = manipulation::workflow::to_ros_duration(failure.elapsed);
    try {
      goal_handle->abort(result);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(logger_, "Could not abort unreserved ServoToPose goal: %s", error.what());
    }
    return;
  }

  publish_feedback(active);
  record_event(active, manipulation::recording::EpisodeLifecycleMarker::kAccepted);

  try {
    auto acquired = config_.skill_runtime->try_acquire(resource_owner_id(active->goal_id));
    if (!acquired.acquired() || acquired.read_only()) {
      std::string detail{"required arm control resource is unavailable"};
      if (!acquired.conflicting_resource_id.empty()) {
        detail += ": " + acquired.conflicting_resource_id;
      }
      const std::lock_guard<std::mutex> lock(active->mutex);
      active->result = admission_failure(
        ErrorCode::kResourceUnavailable, std::move(detail), started_at);
      active->feedback.state.phase = manipulation::task::TaskPhase::kFailed;
      active->feedback.state.detail = active->result->detail;
      active->worker_finished = true;
      return;
    }
    active->resource_lease = std::move(acquired.lease);
  } catch (const std::exception & error) {
    const std::lock_guard<std::mutex> lock(active->mutex);
    active->result = admission_failure(
      ErrorCode::kInternalError,
      std::string{"resource admission failed: "} + error.what(), started_at);
    active->feedback.state.phase = manipulation::task::TaskPhase::kFailed;
    active->feedback.state.detail = active->result->detail;
    active->worker_finished = true;
    return;
  } catch (...) {
    const std::lock_guard<std::mutex> lock(active->mutex);
    active->result = admission_failure(
      ErrorCode::kInternalError,
      "resource admission failed with an unknown exception", started_at);
    active->feedback.state.phase = manipulation::task::TaskPhase::kFailed;
    active->feedback.state.detail = active->result->detail;
    active->worker_finished = true;
    return;
  }

  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    active->worker_running = true;
  }
  try {
    worker_ = std::thread([this, active]() {run_task(active);});
  } catch (const std::exception & error) {
    const std::lock_guard<std::mutex> lock(active->mutex);
    active->worker_running = false;
    active->worker_finished = true;
    active->result = admission_failure(
      ErrorCode::kInternalError,
      std::string{"could not start ServoToPose worker: "} + error.what(), started_at);
    active->feedback.state.phase = manipulation::task::TaskPhase::kFailed;
    active->feedback.state.detail = active->result->detail;
    return;
  }
  record_event(active, manipulation::recording::EpisodeLifecycleMarker::kStarted);
}

void ServoToPoseActionServer::handle_pause_request(
  const std::shared_ptr<PauseService::Request> request,
  std::shared_ptr<PauseService::Response> response)
{
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
  }
  if (!active || !pause_request_matches(request->goal_id, active)) {
    response->message = "no matching active ServoToPose goal";
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(active->mutex);
    if (active->worker_finished || active->terminal_delivered) {
      response->message = "ServoToPose goal is already completing";
      return;
    }
  }

  if (request->paused) {
    active->control->request_pause();
  } else {
    active->control->resume();
  }
  response->accepted = true;
  response->message = request->paused ? "pause intent stored" : "resume intent stored";
}

bool ServoToPoseActionServer::pause_request_matches(
  const unique_identifier_msgs::msg::UUID & requested,
  const std::shared_ptr<ActiveGoal> & active) const noexcept
{
  const bool wildcard = std::all_of(
    requested.uuid.begin(), requested.uuid.end(), [](const std::uint8_t byte) {
      return byte == 0U;
    });
  return wildcard || std::equal(
    requested.uuid.begin(), requested.uuid.end(), active->goal_id.begin());
}

}  // namespace manipulation::moveit
