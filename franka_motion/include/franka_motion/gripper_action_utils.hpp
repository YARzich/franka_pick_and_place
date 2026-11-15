// Responsibility: Provides bounded client-side helpers for sending and awaiting gripper actions.

#pragma once

#include <algorithm>
#include <chrono>
#include <future>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "franka_motion/gripper_utils.hpp"
#include "franka_motion/joint_state_monitor.hpp"

namespace franka_motion
{
using GripperCommandAction = control_msgs::action::GripperCommand;
using GripperCommandClient = rclcpp_action::Client<GripperCommandAction>;
using GripperCommandGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommandAction>;

namespace detail
{
inline bool gripperStopRequested(const std::function<bool()> & stop_requested)
{
  return stop_requested && stop_requested();
}

template<typename FutureT>
bool waitForGripperFutureUntil(
  FutureT & future,
  const std::chrono::steady_clock::time_point deadline,
  const std::function<bool()> & stop_requested,
  bool & stop_observed,
  const bool stop_is_terminal)
{
  while (std::chrono::steady_clock::now() < deadline) {
    stop_observed = stop_observed || gripperStopRequested(stop_requested);
    if (stop_observed && stop_is_terminal) {
      return false;
    }
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      return true;
    }
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const auto poll_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::milliseconds(20));
    future.wait_for(std::min(remaining, poll_period));
  }
  stop_observed = stop_observed || gripperStopRequested(stop_requested);
  return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
}

inline void cancelGripperGoal(
  const GripperCommandClient::SharedPtr & client,
  const GripperCommandGoalHandle::SharedPtr & goal_handle,
  const rclcpp::Logger & logger)
{
  auto cancel_future = client->async_cancel_goal(goal_handle);
  if (cancel_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    RCLCPP_ERROR(logger, "Не дождался подтверждения отмены goal хватателя");
  } else {
    const auto response = cancel_future.get();
    if (!response || response->goals_canceling.empty()) {
      RCLCPP_ERROR(logger, "Сервер хватателя не принял отмену goal");
    }
  }
}
}  // namespace detail

/// Wait for a GripperCommand server until a steady-clock deadline.
inline bool waitForGripperActionServer(
  const GripperCommandClient::SharedPtr & client,
  const rclcpp::Logger & logger,
  const std::string & action_name,
  std::chrono::steady_clock::duration timeout,
  const std::function<bool()> & stop_requested = {})
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && (!stop_requested || !stop_requested()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    if (client->wait_for_action_server(std::chrono::milliseconds(200))) {
      return true;
    }
  }

  if (!rclcpp::ok() || (stop_requested && stop_requested())) {
    return false;
  }
  RCLCPP_ERROR(
    logger,
    "Сервер хватателя %s недоступен",
    action_name.c_str());
  return false;
}

/// Send one goal and synchronously validate its semantic result.
///
/// Widths/tolerances are full fingertip metres and max effort uses the action
/// server's configured hardware units. The caller must keep another executor
/// servicing the action client. Goal acceptance and result each receive the
/// full steady timeout. A result timeout triggers best-effort cancellation;
/// false does not prove the gripper stopped or retained/released an object.
///
/// A stalled result is accepted only for a closing command with measured
/// progress from `command_start_width`.
inline bool sendGripperCommand(
  const GripperCommandClient::SharedPtr & client,
  double width,
  const rclcpp::Logger & logger,
  const std::string & label,
  double max_effort = 40.0,
  std::chrono::steady_clock::duration timeout = std::chrono::seconds(5),
  bool allow_stall = false,
  std::optional<double> command_start_width = std::nullopt,
  double stall_progress_threshold = 0.0005,
  double goal_tolerance = 0.004,
  const std::function<bool()> & stop_requested = {})
{
  if (!isValidGripperCommand(width, max_effort)) {
    RCLCPP_ERROR(
      logger,
      "Некорректная команда хватателя %s: width=%.3f, max_effort=%.3f",
      label.c_str(), width, max_effort);
    return false;
  }

  auto goal = GripperCommandAction::Goal();
  goal.command.position = clampGripperWidth(width);
  goal.command.max_effort = max_effort;

  RCLCPP_INFO(
    logger,
    "Команда хватателя %s: ширина %.3f м",
    label.c_str(),
    goal.command.position);
  if (detail::gripperStopRequested(stop_requested)) {
    return false;
  }
  const auto goal_deadline = std::chrono::steady_clock::now() + timeout;
  auto future_goal = client->async_send_goal(goal);
  bool stop_observed = false;
  if (!detail::waitForGripperFutureUntil(
      future_goal, goal_deadline, stop_requested, stop_observed, false))
  {
    RCLCPP_ERROR(logger, "Не дождался отправки goal хватателя");
    return false;
  }

  auto goal_handle = future_goal.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Хвататель отклонил goal");
    return false;
  }
  if (stop_observed || detail::gripperStopRequested(stop_requested)) {
    detail::cancelGripperGoal(client, goal_handle, logger);
    return false;
  }

  const auto result_deadline = std::chrono::steady_clock::now() + timeout;
  auto result_future = client->async_get_result(goal_handle);
  stop_observed = false;
  if (!detail::waitForGripperFutureUntil(
      result_future, result_deadline, stop_requested, stop_observed, true))
  {
    if (stop_observed) {
      detail::cancelGripperGoal(client, goal_handle, logger);
      return false;
    }
    RCLCPP_ERROR(logger, "Не дождался результата хватателя");
    detail::cancelGripperGoal(client, goal_handle, logger);
    return false;
  }
  if (stop_observed || detail::gripperStopRequested(stop_requested)) {
    detail::cancelGripperGoal(client, goal_handle, logger);
    return false;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    const char * code = "UNKNOWN";
    if (wrapped_result.code == rclcpp_action::ResultCode::ABORTED) {
      code = "ABORTED";
    } else if (wrapped_result.code == rclcpp_action::ResultCode::CANCELED) {
      code = "CANCELED";
    }
    RCLCPP_ERROR(
      logger,
      "Команда хватателя %s завершилась со статусом %s",
      label.c_str(),
      code);
    return false;
  }
  if (!wrapped_result.result) {
    RCLCPP_ERROR(
      logger,
      "Сервер хватателя вернул пустой результат для %s",
      label.c_str());
    return false;
  }
  const bool accepted_stall = allow_stall && wrapped_result.result->stalled &&
    command_start_width.has_value() && gripperStallHasClosingProgress(
    command_start_width.value(), wrapped_result.result->position, goal.command.position,
    stall_progress_threshold, goal_tolerance);
  if (!wrapped_result.result->reached_goal && !accepted_stall) {
    RCLCPP_ERROR(
      logger,
      "Хвататель не выполнил %s: position=%.3f, reached_goal=%s, stalled=%s",
      label.c_str(), wrapped_result.result->position,
      wrapped_result.result->reached_goal ? "true" : "false",
      wrapped_result.result->stalled ? "true" : "false");
    return false;
  }

  RCLCPP_INFO(logger, "Хвататель выполнил команду %s", label.c_str());
  return true;
}

/// Steady deadlines and metre thresholds for `sendGripperCommandAndWait()`.
struct GripperCommandWaitOptions
{
  std::chrono::steady_clock::duration action_timeout{std::chrono::seconds(5)};
  std::chrono::steady_clock::duration state_timeout{std::chrono::milliseconds(800)};
  std::chrono::steady_clock::duration max_state_age{std::chrono::milliseconds(500)};
  double max_effort{40.0};
  double position_tolerance{0.002};
  double stall_progress_threshold{0.0005};
  double goal_tolerance{0.004};
};

/// Send a goal, validate its action result, then require fresh measured finger state.
///
/// This blocks the caller while another executor services action and JointState
/// callbacks. False leaves the physical grasp disposition unconfirmed.
inline bool sendGripperCommandAndWait(
  const GripperCommandClient::SharedPtr & client,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  double target_width,
  const std::string & action_label,
  const std::string & state_label,
  bool allow_stall,
  const GripperCommandWaitOptions & options,
  const std::function<bool()> & stop_requested = {})
{
  const auto initial_state = joint_state_monitor.freshGripperState(options.max_state_age);
  const std::optional<double> command_start_width = initial_state.has_value() ?
    std::optional<double>(initial_state->left + initial_state->right) : std::nullopt;
  if (!sendGripperCommand(
      client,
      target_width,
      logger,
      action_label,
      options.max_effort,
      options.action_timeout,
      allow_stall,
      command_start_width,
      options.stall_progress_threshold,
      options.goal_tolerance,
      stop_requested))
  {
    return false;
  }

  return joint_state_monitor.waitForGripperWidth(
    target_width,
    options.state_timeout,
    options.position_tolerance,
    allow_stall,
    state_label,
    logger,
    options.max_state_age,
    command_start_width,
    options.stall_progress_threshold,
    stop_requested);
}
}  // namespace franka_motion
