// Responsibility: Executes one ROS action call to a steady deadline with cooperative cancellation.

#ifndef ACTION_CALL_HPP_
#define ACTION_CALL_HPP_

#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/context.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_call_state.hpp"
#include "manipulation_moveit/arm_planning_port.hpp"

namespace manipulation::moveit::detail
{

/**
 * One sentinel governs discovery, goal response, and result waiting.
 *
 * Keeping this as an explicit policy seam prevents the removed 5-second
 * discovery and 60-second response limits from returning unnoticed.
 */
constexpr std::chrono::steady_clock::time_point unbounded_action_deadline() noexcept
{
  return std::chrono::steady_clock::time_point::max();
}

/**
 * Send one action goal and wait for its terminal result under a steady deadline.
 *
 * The worker thread never spins; executor callbacks retain shared call state so
 * a late goal response remains owned after timeout. `active_slot` prevents a
 * later goal from overlapping that unresolved response. Stop requests cancel
 * only the exact accepted goal and do not imply immediate physical termination.
 * A plan-only caller may return on the stop request while the active slot keeps
 * the late terminal result quarantined; physical callers retain the default
 * wait-for-terminal behavior.
 * If send itself throws, its outcome is not documented by rclcpp; the active
 * slot therefore remains quarantined until a late reject/result or node restart.
 */
template<typename ActionT>
ActionCallResult<ActionT> call_action_until(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const typename ActionT::Goal & goal,
  const std::chrono::steady_clock::time_point deadline,
  const std::chrono::steady_clock::duration poll_period,
  const std::shared_ptr<rclcpp::Context> & context,
  const PlanningStopToken & stop_token,
  const std::function<bool()> & adapter_stop_requested,
  ActiveActionSlot & active_slot,
  std::mutex & dispatch_mutex,
  const bool return_on_stop_request = false)
{
  if (!context || !context->is_valid()) {
    return {ActionCallStatus::kShutdown, std::nullopt, "ROS context is shutting down"};
  }
  if (stop_token.checkpoint() != PlanningStopCause::kNone) {
    return {ActionCallStatus::kCanceled, std::nullopt, "action canceled before send"};
  }
  while (!client->action_server_is_ready()) {
    if (stop_token.checkpoint() != PlanningStopCause::kNone) {
      return {ActionCallStatus::kCanceled, std::nullopt, "action canceled before send"};
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return {ActionCallStatus::kUnavailable, std::nullopt, "action server is unavailable"};
    }
    try {
      (void)client->wait_for_action_server(std::min(poll_period, deadline - now));
    } catch (const std::exception & error) {
      return {ActionCallStatus::kInternalError, std::nullopt, error.what()};
    }
    if (!context->is_valid()) {
      return {ActionCallStatus::kShutdown, std::nullopt, "ROS context is shutting down"};
    }
  }
  if (stop_token.checkpoint() != PlanningStopCause::kNone) {
    return {ActionCallStatus::kCanceled, std::nullopt, "action canceled before send"};
  }
  if (std::chrono::steady_clock::now() >= deadline) {
    return {ActionCallStatus::kTimeout, std::nullopt, "action deadline expired before send"};
  }

  const auto state = std::make_shared<ActionCallState<ActionT>>(client);
  bool installed = false;
  bool stopped_inside_gate = false;
  while (!installed) {
    const auto gate = stop_token.run_operation(
      [&]() {
        const std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex);
        // The lifecycle gate already owns its non-recursive session mutex.
        // Only adapter-local state may be queried from inside this closure.
        if (adapter_stop_requested && adapter_stop_requested()) {
          stopped_inside_gate = true;
          return;
        }
        installed = active_slot.install(state);
        if (!installed) {
          return;
        }
        try {
          (void)client->async_send_goal(goal, state->callbacks());
        } catch (const std::exception & error) {
          state->mark_send_failure(error.what());
          state->request_cancel();
        } catch (...) {
          state->mark_send_failure("unknown exception while sending action goal");
          state->request_cancel();
        }
      });
    if (gate == manipulation::workflow::TaskOperationGateResult::kStopped ||
      stopped_inside_gate)
    {
      return {
        ActionCallStatus::kCanceled, std::nullopt,
        "action dispatch was stopped before send"};
    }
    if (gate == manipulation::workflow::TaskOperationGateResult::kPaused) {
      if (stop_token.checkpoint() != PlanningStopCause::kNone) {
        return {
          ActionCallStatus::kCanceled, std::nullopt,
          "action dispatch was stopped while paused"};
      }
      continue;
    }
    if (!installed) {
      return {
        ActionCallStatus::kUnavailable, std::nullopt,
        "previous action goal has not reached a terminal state"};
    }
  }
  if (stop_token.stop_requested() ||
    (adapter_stop_requested && adapter_stop_requested()))
  {
    state->request_cancel();
  }
  return state->wait_until(
    deadline, poll_period, context,
    [&stop_token, &adapter_stop_requested]() {
      return stop_token.stop_requested() ||
             (adapter_stop_requested && adapter_stop_requested());
    },
    [&stop_token]() {
      return stop_token.heartbeat() != PlanningStopCause::kNone;
    },
    return_on_stop_request);
}

}  // namespace manipulation::moveit::detail

#endif  // ACTION_CALL_HPP_
