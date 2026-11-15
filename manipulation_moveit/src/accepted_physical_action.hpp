// Responsibility: Owns acceptance-relative ROS action timing and exact-goal stop confirmation.

#ifndef ACCEPTED_PHYSICAL_ACTION_HPP_
#define ACCEPTED_PHYSICAL_ACTION_HPP_

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/context.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_call_state.hpp"
#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace manipulation::moveit::detail
{

// Private responsiveness and stop-evidence windows; neither is deployment policy.
constexpr auto kPhysicalActionWaitSlice = std::chrono::milliseconds{5};
constexpr auto kPhysicalStopConfirmationWindow = std::chrono::milliseconds{200};

enum class AcceptedPhysicalActionStatus
{
  kResultReceived,
  kRejected,
  kCanceled,
  kTimedOut,
  kUnavailable,
  kShutdown,
  kInternalError,
};

template<typename ActionT>
struct AcceptedPhysicalActionResult
{
  AcceptedPhysicalActionStatus status{AcceptedPhysicalActionStatus::kInternalError};
  std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult> result;
  std::string detail{"accepted physical action result was not initialized"};
  ActionDispatchState dispatch_state{ActionDispatchState::kNotSent};
  bool terminal_confirmed{false};
  std::optional<std::chrono::steady_clock::time_point> accepted_deadline;
};

template<typename ActionT>
class AcceptedPhysicalActionState final
  : public CancelableActionCall,
  public std::enable_shared_from_this<AcceptedPhysicalActionState<ActionT>>
{
public:
  using Clock = std::chrono::steady_clock;
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  AcceptedPhysicalActionState(
    typename Client::SharedPtr client,
    const Clock::duration accepted_timeout)
  : client_(std::move(client)), accepted_timeout_(accepted_timeout)
  {
  }

  typename Client::SendGoalOptions callbacks()
  {
    const auto self = this->shared_from_this();
    typename Client::SendGoalOptions options;
    options.goal_response_callback = [self](typename GoalHandle::SharedPtr handle) {
        self->on_goal_response(std::move(handle));
      };
    options.result_callback = [self](const WrappedResult & result) {
        self->on_result(result);
      };
    return options;
  }

  void mark_send_failure(std::string detail) noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    send_failed_ = true;
    cancel_requested_ = true;
    dispatch_state_ = ActionDispatchState::kSendOutcomeUnknown;
    detail_ = std::move(detail);
    cv_.notify_all();
  }

  void request_cancel() noexcept override
  {
    typename GoalHandle::SharedPtr handle;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cancel_requested_ = true;
      if (goal_handle_ && !result_ && !cancel_sent_) {
        cancel_sent_ = true;
        handle = goal_handle_;
      }
      cv_.notify_all();
    }
    if (handle) {
      dispatch_cancel(std::move(handle));
    }
  }

  [[nodiscard]] bool terminal() const noexcept override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return result_.has_value() || (goal_response_received_ && !goal_handle_);
  }

  [[nodiscard]] bool dispatch_unknown() const noexcept override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return send_failed_ && !result_;
  }

  [[nodiscard]] AcceptedPhysicalActionResult<ActionT> wait(
    const std::shared_ptr<rclcpp::Context> & context,
    manipulation::workflow::TransferExecutionContext & execution_context,
    const std::function<bool()> & stop_requested)
  {
    bool maintenance_stopped = false;
    std::unique_lock<std::mutex> lock(mutex_);
    for (;; ) {
      if (result_) {
        const bool exceeded_budget = accepted_deadline_ && result_received_at_ &&
          *result_received_at_ > *accepted_deadline_;
        return {
          exceeded_budget ? AcceptedPhysicalActionStatus::kTimedOut :
          AcceptedPhysicalActionStatus::kResultReceived,
          result_,
          exceeded_budget ?
          "physical action reached terminal state after its accepted-command budget" : detail_,
          ActionDispatchState::kAccepted,
          true,
          accepted_deadline_};
      }
      if (goal_response_received_ && !goal_handle_) {
        return {
          cancel_requested_ ? AcceptedPhysicalActionStatus::kCanceled :
          AcceptedPhysicalActionStatus::kRejected,
          std::nullopt,
          cancel_requested_ ?
          "physical action was rejected after cancellation" : "physical action was rejected",
          ActionDispatchState::kRejected,
          true};
      }
      if (send_failed_) {
        return {
          AcceptedPhysicalActionStatus::kInternalError, std::nullopt, detail_,
          ActionDispatchState::kSendOutcomeUnknown, false};
      }
      if (!context || !context->is_valid()) {
        return stop_and_confirm(
          lock, AcceptedPhysicalActionStatus::kShutdown,
          "ROS context shut down during physical action");
      }
      if (maintenance_stopped || stop_requested()) {
        return stop_and_confirm(
          lock, AcceptedPhysicalActionStatus::kCanceled,
          "physical action was canceled");
      }

      const auto now = Clock::now();
      if (accepted_deadline_ && now >= *accepted_deadline_) {
        return stop_and_confirm(
          lock, AcceptedPhysicalActionStatus::kTimedOut,
          "accepted physical command exceeded its motion budget");
      }
      if (accepted_deadline_) {
        cv_.wait_until(lock, std::min(*accepted_deadline_, now + kPhysicalActionWaitSlice));
      } else {
        cv_.wait_for(lock, kPhysicalActionWaitSlice);
      }
      lock.unlock();
      maintenance_stopped =
        execution_context.heartbeat() ==
        manipulation::workflow::TransferExecutionControlResult::kStopped;
      lock.lock();
    }
  }

private:
  static Clock::time_point accepted_deadline(
    const Clock::time_point accepted_at,
    const Clock::duration timeout) noexcept
  {
    if (timeout >= Clock::time_point::max() - accepted_at) {
      return Clock::time_point::max();
    }
    return accepted_at + timeout;
  }

  AcceptedPhysicalActionResult<ActionT> stop_and_confirm(
    std::unique_lock<std::mutex> & lock,
    const AcceptedPhysicalActionStatus status,
    std::string detail)
  {
    lock.unlock();
    request_cancel();
    lock.lock();
    const auto confirmation_deadline = Clock::now() + kPhysicalStopConfirmationWindow;
    while (!result_ && !(goal_response_received_ && !goal_handle_) &&
      Clock::now() < confirmation_deadline)
    {
      cv_.wait_until(lock, confirmation_deadline);
    }
    if (result_) {
      return {
        status, result_, detail + "; terminal state confirmed",
        ActionDispatchState::kAccepted, true, accepted_deadline_};
    }
    if (goal_response_received_ && !goal_handle_) {
      return {
        status, std::nullopt, detail + "; goal was not accepted",
        ActionDispatchState::kRejected, true};
    }
    return {status, std::nullopt, std::move(detail), dispatch_state_, false};
  }

  void on_goal_response(typename GoalHandle::SharedPtr handle) noexcept
  {
    typename GoalHandle::SharedPtr cancel_handle;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      goal_response_received_ = true;
      goal_handle_ = std::move(handle);
      dispatch_state_ = goal_handle_ ?
        ActionDispatchState::kAccepted : ActionDispatchState::kRejected;
      if (goal_handle_) {
        const auto accepted_at = Clock::now();
        accepted_deadline_ = accepted_deadline(accepted_at, accepted_timeout_);
      }
      if (goal_handle_ && cancel_requested_ && !cancel_sent_) {
        cancel_sent_ = true;
        cancel_handle = goal_handle_;
      }
      cv_.notify_all();
    }
    if (cancel_handle) {
      dispatch_cancel(std::move(cancel_handle));
    }
  }

  void on_result(const WrappedResult & result) noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    dispatch_state_ = ActionDispatchState::kAccepted;
    result_ = result;
    result_received_at_ = Clock::now();
    cv_.notify_all();
  }

  void dispatch_cancel(typename GoalHandle::SharedPtr handle) noexcept
  {
    try {
      const auto self = this->shared_from_this();
      (void)client_->async_cancel_goal(
        std::move(handle), [self](const auto) {self->notify_waiter();});
    } catch (const std::exception & error) {
      record_cancel_error(error.what());
    } catch (...) {
      record_cancel_error("unknown exception while canceling physical action");
    }
  }

  void notify_waiter() noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    cv_.notify_all();
  }

  void record_cancel_error(std::string detail) noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    detail_ = std::move(detail);
    cv_.notify_all();
  }

  typename Client::SharedPtr client_;
  const Clock::duration accepted_timeout_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::optional<WrappedResult> result_;
  std::optional<Clock::time_point> result_received_at_;
  std::optional<Clock::time_point> accepted_deadline_;
  bool goal_response_received_{false};
  bool cancel_requested_{false};
  bool cancel_sent_{false};
  bool send_failed_{false};
  std::string detail_;
  ActionDispatchState dispatch_state_{ActionDispatchState::kAwaitingGoalResponse};
};

template<typename ActionT>
AcceptedPhysicalActionResult<ActionT> call_accepted_physical_action(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const typename ActionT::Goal & goal,
  const std::chrono::steady_clock::duration accepted_timeout,
  const std::shared_ptr<rclcpp::Context> & context,
  manipulation::workflow::TransferExecutionContext & execution_context,
  const std::function<bool()> & stop_requested,
  ActiveActionSlot & active_slot,
  std::mutex & dispatch_mutex)
{
  if (!context || !context->is_valid()) {
    return {
      AcceptedPhysicalActionStatus::kShutdown, std::nullopt,
      "ROS context is shutting down", ActionDispatchState::kNotSent, false};
  }
  if (stop_requested() || execution_context.stop_requested()) {
    return {
      AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
      "physical action canceled before discovery", ActionDispatchState::kNotSent, false};
  }
  while (!client->action_server_is_ready()) {
    if (execution_context.checkpoint() ==
      manipulation::workflow::TransferExecutionControlResult::kStopped ||
      stop_requested())
    {
      return {
        AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
        "physical action canceled during discovery", ActionDispatchState::kNotSent, false};
    }
    if (!context->is_valid()) {
      return {
        AcceptedPhysicalActionStatus::kShutdown, std::nullopt,
        "ROS context shut down during discovery", ActionDispatchState::kNotSent, false};
    }
    try {
      (void)client->wait_for_action_server(kPhysicalActionWaitSlice);
    } catch (const std::exception & error) {
      return {
        AcceptedPhysicalActionStatus::kInternalError, std::nullopt, error.what(),
        ActionDispatchState::kNotSent, false};
    }
  }
  if (execution_context.checkpoint() ==
    manipulation::workflow::TransferExecutionControlResult::kStopped ||
    stop_requested())
  {
    return {
      AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
      "physical action canceled before send", ActionDispatchState::kNotSent, false};
  }

  const auto state = std::make_shared<AcceptedPhysicalActionState<ActionT>>(
    client, accepted_timeout);
  bool installed = false;
  bool stopped_inside_gate = false;
  while (!installed) {
    const auto gate = execution_context.run_operation(
      [&]() {
        const std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex);
        // run_operation() already owns the lifecycle session mutex. Querying
        // the execution context here would re-enter that non-recursive mutex.
        if (stop_requested()) {
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
          state->mark_send_failure("unknown exception while sending physical action");
          state->request_cancel();
        }
      });
    if (gate == manipulation::workflow::TaskOperationGateResult::kStopped ||
      stopped_inside_gate)
    {
      return {
        AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
        "physical action dispatch was stopped before send",
        ActionDispatchState::kNotSent, false};
    }
    if (gate == manipulation::workflow::TaskOperationGateResult::kPaused) {
      if (execution_context.checkpoint() ==
        manipulation::workflow::TransferExecutionControlResult::kStopped)
      {
        return {
          AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
          "physical action dispatch was stopped while paused",
          ActionDispatchState::kNotSent, false};
      }
      continue;
    }
    if (!installed) {
      return {
        AcceptedPhysicalActionStatus::kUnavailable, std::nullopt,
        "previous physical action has not reached a terminal state",
        ActionDispatchState::kAccepted, false};
    }
  }
  return state->wait(context, execution_context, stop_requested);
}

template<typename ActionT>
PhysicalCommandSafety physical_action_safety(
  const AcceptedPhysicalActionResult<ActionT> & result) noexcept
{
  if (result.dispatch_state == ActionDispatchState::kNotSent ||
    result.dispatch_state == ActionDispatchState::kRejected)
  {
    return PhysicalCommandSafety::kNotAccepted;
  }
  return result.terminal_confirmed ?
         PhysicalCommandSafety::kTerminalSafe : PhysicalCommandSafety::kActiveStateUnknown;
}

}  // namespace manipulation::moveit::detail

#endif  // ACCEPTED_PHYSICAL_ACTION_HPP_
