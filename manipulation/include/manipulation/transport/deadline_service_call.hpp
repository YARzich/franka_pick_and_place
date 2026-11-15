// Responsibility: Executes one typed ROS 2 service request under an absolute steady deadline.

#ifndef MANIPULATION__TRANSPORT__DEADLINE_SERVICE_CALL_HPP_
#define MANIPULATION__TRANSPORT__DEADLINE_SERVICE_CALL_HPP_

#include <algorithm>
#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace manipulation::transport
{

enum class ServiceCallStatus
{
  kSuccess,
  kCanceled,
  kUnavailable,
  kShutdown,
  kTimeout,
  kInternalError,
};

template<typename ServiceT>
struct ServiceCallResult
{
  ServiceCallStatus status{ServiceCallStatus::kInternalError};
  typename ServiceT::Response::SharedPtr response;
  std::string detail;
  /// True after async_send_request returned a future for this request.
  bool request_sent{false};
};

/**
 * Wait for a ROS service and response under one absolute steady deadline.
 *
 * The calling thread never spins an executor. A MultiThreadedExecutor must own
 * the client's callback group so its response waitable can complete the
 * future. `context` must be the context that owns the client. Timeout, cancel,
 * and shutdown paths remove an already-sent request from rclcpp's pending map.
 */
template<typename ServiceT, typename StopPredicate>
ServiceCallResult<ServiceT> call_service_until(
  rclcpp::Client<ServiceT> & client,
  typename ServiceT::Request::SharedPtr request,
  const std::chrono::steady_clock::time_point deadline,
  const std::chrono::steady_clock::duration poll_period,
  const std::shared_ptr<rclcpp::Context> & context,
  StopPredicate stop_requested)
{
  using Clock = std::chrono::steady_clock;
  if (!request || poll_period <= Clock::duration::zero()) {
    return {ServiceCallStatus::kInternalError, {}, "invalid service call arguments", false};
  }
  if (!context) {
    return {ServiceCallStatus::kShutdown, {}, "ROS context is unavailable", false};
  }
  if (!context->is_valid()) {
    return {ServiceCallStatus::kShutdown, {}, "ROS context is shutting down", false};
  }

  while (true) {
    if (!context->is_valid()) {
      return {
        ServiceCallStatus::kShutdown, {},
        "ROS context shut down during service wait", false};
    }
    try {
      if (client.service_is_ready()) {
        break;
      }
    } catch (const std::exception & error) {
      if (!context->is_valid()) {
        return {
          ServiceCallStatus::kShutdown, {},
          "ROS context shut down during service wait", false};
      }
      return {
        ServiceCallStatus::kInternalError, {},
        std::string{"service readiness check threw: "} + error.what(), false};
    }
    if (stop_requested()) {
      return {ServiceCallStatus::kCanceled, {}, "service wait canceled", false};
    }
    const auto now = Clock::now();
    if (now >= deadline) {
      return {ServiceCallStatus::kUnavailable, {}, "service unavailable at deadline", false};
    }
    try {
      (void)client.wait_for_service(std::min(poll_period, deadline - now));
    } catch (const std::exception & error) {
      if (!context->is_valid()) {
        return {
          ServiceCallStatus::kShutdown, {},
          "ROS context shut down during service wait", false};
      }
      return {
        ServiceCallStatus::kInternalError, {},
        std::string{"service discovery threw: "} + error.what(), false};
    }
  }
  if (!context->is_valid()) {
    return {
      ServiceCallStatus::kShutdown, {},
      "ROS context shut down before service send", false};
  }
  if (stop_requested()) {
    return {ServiceCallStatus::kCanceled, {}, "service call canceled before send", false};
  }
  if (Clock::now() >= deadline) {
    return {
      ServiceCallStatus::kTimeout, {},
      "service deadline expired before send", false};
  }

  bool request_sent = false;
  try {
    auto future = client.async_send_request(std::move(request));
    request_sent = true;
    while (future.wait_for(Clock::duration::zero()) != std::future_status::ready) {
      if (!context->is_valid()) {
        client.remove_pending_request(future);
        return {
          ServiceCallStatus::kShutdown, {},
          "ROS context shut down during service response wait", true};
      }
      if (stop_requested()) {
        client.remove_pending_request(future);
        return {ServiceCallStatus::kCanceled, {}, "service response wait canceled", true};
      }
      const auto now = Clock::now();
      if (now >= deadline) {
        client.remove_pending_request(future);
        return {ServiceCallStatus::kTimeout, {}, "service response deadline exceeded", true};
      }
      (void)future.wait_for(std::min(poll_period, deadline - now));
    }
    auto response = future.get();
    if (!response) {
      return {ServiceCallStatus::kInternalError, {}, "service returned a null response", true};
    }
    return {ServiceCallStatus::kSuccess, std::move(response), {}, true};
  } catch (const std::exception & error) {
    return {
      ServiceCallStatus::kInternalError, {},
      std::string{"service call threw: "} + error.what(), request_sent};
  } catch (...) {
    return {
      ServiceCallStatus::kInternalError, {},
      "service call threw an unknown exception", request_sent};
  }
}

}  // namespace manipulation::transport

#endif  // MANIPULATION__TRANSPORT__DEADLINE_SERVICE_CALL_HPP_
