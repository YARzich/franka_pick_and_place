// Responsibility: Owns action goal/result state and arbitrates cancellation across callback races.

#ifndef ACTION_CALL_STATE_HPP_
#define ACTION_CALL_STATE_HPP_

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

namespace manipulation::moveit::detail
{

enum class ActionCallStatus
{
  kResultReceived,
  kRejected,
  kCanceled,
  kTimeout,
  kUnavailable,
  kShutdown,
  kInternalError,
};

/// Best-known dispatch boundary for an action request when the worker stops waiting.
enum class ActionDispatchState
{
  kNotSent,
  kSendOutcomeUnknown,
  kAwaitingGoalResponse,
  kRejected,
  kAccepted,
};

template<typename ActionT>
struct ActionCallResult
{
  ActionCallStatus status{ActionCallStatus::kInternalError};
  std::optional<typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult> result;
  std::string detail{"action call result was not initialized"};
  ActionDispatchState dispatch_state{ActionDispatchState::kNotSent};
};

class CancelableActionCall
{
public:
  virtual ~CancelableActionCall() = default;
  virtual void request_cancel() noexcept = 0;
  [[nodiscard]] virtual bool terminal() const noexcept = 0;
  [[nodiscard]] virtual bool dispatch_unknown() const noexcept = 0;
};

/** Thread-safe owner slot preventing overlap with a late action response. */
class ActiveActionSlot final
{
public:
  [[nodiscard]] bool install(const std::shared_ptr<CancelableActionCall> & call)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && !active_->terminal()) {
      return false;
    }
    active_ = call;
    return true;
  }

  void request_cancel() noexcept
  {
    std::shared_ptr<CancelableActionCall> active;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      active = active_;
    }
    if (active) {
      active->request_cancel();
    }
  }

  [[nodiscard]] bool busy() const noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return active_ && !active_->terminal();
  }

  [[nodiscard]] bool dispatch_unknown() const noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return active_ && active_->dispatch_unknown();
  }

private:
  mutable std::mutex mutex_;
  std::shared_ptr<CancelableActionCall> active_;
};

/**
 * Shared callback state for one action request whose response may outlive a worker wait.
 *
 * Executor callbacks and the worker may call all methods concurrently. The
 * client and this state own the exact goal handle until a terminal response;
 * cancellation is dispatched outside `mutex_` to avoid callback lock inversion.
 */
template<typename ActionT>
class ActionCallState final
  : public CancelableActionCall,
  public std::enable_shared_from_this<ActionCallState<ActionT>>
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  explicit ActionCallState(typename Client::SharedPtr client)
  : client_(std::move(client)) {}

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
    // An exception from async_send_goal() does not prove that no request left
    // the process. Keep the slot reserved until a late reject/result proves a
    // terminal state; otherwise a new physical goal could overlap it.
    return result_.has_value() ||
           (goal_response_received_ && !goal_handle_);
  }

  [[nodiscard]] bool dispatch_unknown() const noexcept override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return send_failed_ &&
           dispatch_state_ == ActionDispatchState::kSendOutcomeUnknown &&
           !result_;
  }

  [[nodiscard]] ActionCallResult<ActionT> wait_until(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::duration poll_period,
    const std::shared_ptr<rclcpp::Context> & context,
    const std::function<bool()> & stop_requested,
    const std::function<bool()> & heartbeat_stopped,
    const bool return_on_stop_request = false)
  {
    bool cancellation_observed = false;
    std::unique_lock<std::mutex> lock(mutex_);
    for (;; ) {
      if (result_) {
        return {
          ActionCallStatus::kResultReceived, result_, detail_,
          ActionDispatchState::kAccepted};
      }
      if (goal_response_received_ && !goal_handle_) {
        return {
          cancel_requested_ ? ActionCallStatus::kCanceled : ActionCallStatus::kRejected,
          std::nullopt,
          cancel_requested_ ? "action goal rejected after cancellation was requested" :
          "action goal rejected",
          ActionDispatchState::kRejected};
      }
      if (send_failed_) {
        return {
          ActionCallStatus::kInternalError, std::nullopt, detail_,
          ActionDispatchState::kSendOutcomeUnknown};
      }
      if (!context || !context->is_valid()) {
        lock.unlock();
        request_cancel();
        lock.lock();
        return {
          ActionCallStatus::kShutdown, std::nullopt, "ROS context is shutting down",
          dispatch_state_locked()};
      }
      if (!cancellation_observed && (cancel_requested_ || stop_requested())) {
        cancellation_observed = true;
        lock.unlock();
        request_cancel();
        lock.lock();
        if (return_on_stop_request) {
          return {
            ActionCallStatus::kCanceled, std::nullopt,
            "action cancellation requested; late terminal state remains owned",
            dispatch_state_locked()};
        }
        continue;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        lock.unlock();
        request_cancel();
        lock.lock();
        return {
          cancellation_observed ? ActionCallStatus::kCanceled : ActionCallStatus::kTimeout,
          std::nullopt,
          cancellation_observed ? "action cancellation did not reach a terminal result" :
          "action result exceeded the absolute deadline",
          dispatch_state_locked()};
      }
      cv_.wait_until(lock, std::min(deadline, now + poll_period));
      lock.unlock();
      const bool maintenance_stopped =
        heartbeat_stopped && heartbeat_stopped();
      lock.lock();
      if (maintenance_stopped && !cancellation_observed) {
        cancellation_observed = true;
        lock.unlock();
        request_cancel();
        lock.lock();
        if (return_on_stop_request) {
          return {
            ActionCallStatus::kCanceled, std::nullopt,
            "action maintenance stopped; late terminal state remains owned",
            dispatch_state_locked()};
        }
      }
    }
  }

private:
  void on_goal_response(typename GoalHandle::SharedPtr handle) noexcept
  {
    typename GoalHandle::SharedPtr cancel_handle;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      goal_response_received_ = true;
      goal_handle_ = std::move(handle);
      dispatch_state_ = goal_handle_ ?
        ActionDispatchState::kAccepted : ActionDispatchState::kRejected;
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
      record_cancel_error("unknown exception while canceling action goal");
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

  [[nodiscard]] ActionDispatchState dispatch_state_locked() const noexcept
  {
    return dispatch_state_;
  }

  typename Client::SharedPtr client_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::optional<WrappedResult> result_;
  bool goal_response_received_{false};
  bool cancel_requested_{false};
  bool cancel_sent_{false};
  bool send_failed_{false};
  std::string detail_;
  ActionDispatchState dispatch_state_{ActionDispatchState::kAwaitingGoalResponse};
};

}  // namespace manipulation::moveit::detail

#endif  // ACTION_CALL_STATE_HPP_
