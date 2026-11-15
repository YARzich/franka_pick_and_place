// Responsibility: Polls controller and PlanningScene services with bounded request lifetimes.

#include "system_health_monitor.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace manipulation::monitoring
{

void SystemHealthMonitor::pollControllers()
{
  if (!controller_client_->service_is_ready()) {
    return;
  }
  const auto token = controller_request_tracker_.tryStart(
    FreshnessMonitor::Clock::now(), service_request_timeout_);
  if (!token) {
    return;
  }

  auto request = std::make_shared<ListControllers::Request>();
  try {
    const auto request_handle = controller_client_->async_send_request(
      request,
      [this, token = *token](rclcpp::Client<ListControllers>::SharedFuture future) {
        const auto disposition = controller_request_tracker_.acceptResponse(
          token, FreshnessMonitor::Clock::now());
        if (disposition == detail::ServiceResponseDisposition::kStaleGeneration) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring late ListControllers response from an expired request generation");
          return;
        }
        if (disposition == detail::ServiceResponseDisposition::kDeadlineExceeded) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring ListControllers response received after the steady request deadline");
          return;
        }
        try {
          const auto response = future.get();
          std::unordered_map<std::string, std::string> states;
          for (const auto & controller : response->controller) {
            states.emplace(controller.name, controller.state);
          }
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            controller_states_ = std::move(states);
          }
          // Observe after releasing data_mutex_ to preserve a single lock order.
          controller_monitor_->observe();
        } catch (const std::exception & exception) {
          RCLCPP_WARN(get_logger(), "ListControllers failed: %s", exception.what());
        }
      });
    if (!controller_request_tracker_.bindRequestId(*token, request_handle.request_id)) {
      (void)controller_client_->remove_pending_request(request_handle.request_id);
    }
  } catch (const std::exception & exception) {
    (void)controller_request_tracker_.abandon(*token);
    RCLCPP_WARN(get_logger(), "Cannot request controller state: %s", exception.what());
  }
}

void SystemHealthMonitor::pollPlanningScene()
{
  if (!planning_scene_client_->service_is_ready()) {
    return;
  }
  const auto token = planning_scene_request_tracker_.tryStart(
    FreshnessMonitor::Clock::now(), service_request_timeout_);
  if (!token) {
    return;
  }

  auto request = std::make_shared<GetPlanningSceneState::Request>();
  try {
    const auto request_handle = planning_scene_client_->async_send_request(
      request,
      [this, token = *token](rclcpp::Client<GetPlanningSceneState>::SharedFuture future) {
        const auto disposition = planning_scene_request_tracker_.acceptResponse(
          token, FreshnessMonitor::Clock::now());
        if (disposition == detail::ServiceResponseDisposition::kStaleGeneration) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring late PlanningScene state response from an expired request generation");
          return;
        }
        if (disposition == detail::ServiceResponseDisposition::kDeadlineExceeded) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring PlanningScene state response received after the steady request deadline");
          return;
        }
        try {
          const auto response = future.get();
          if (!response->result.success) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "PlanningScene state request failed with code %u: %s",
              static_cast<unsigned int>(response->result.error_code),
              response->result.message.c_str());
          } else if (response->state.session_id.empty()) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "PlanningScene coordinator returned an empty session_id");
          } else {
            const auto observed_at = FreshnessMonitor::Clock::now();
            {
              std::lock_guard<std::mutex> lock(data_mutex_);
              const bool session_changed =
              !planning_scene_session_id_.empty() &&
              planning_scene_session_id_ != response->state.session_id;
              if (response->state.operation_in_progress) {
                if (!planning_scene_operation_in_progress_ || session_changed) {
                  planning_scene_operation_started_at_ = observed_at;
                }
              } else {
                planning_scene_operation_started_at_.reset();
              }
              planning_scene_session_id_ = response->state.session_id;
              planning_scene_revision_ = response->state.revision;
              planning_scene_operation_in_progress_ =
              response->state.operation_in_progress;
              planning_scene_backend_ready_ = response->state.backend_service_ready;
              planning_scene_reconciliation_required_ =
              response->state.reconciliation_required;
              planning_scene_backend_error_code_ =
              response->state.last_backend_error_code;
              planning_scene_expired_lease_count_ =
              response->state.expired_lease_count;
              planning_scene_unreconciled_lease_expiry_count_ =
              response->state.unreconciled_lease_expiry_count;
              planning_scene_operation_capacity_exhausted_ =
              response->state.operation_capacity_exhausted;
            }
            // Observe after releasing data_mutex_ to preserve a single lock order.
            planning_scene_monitor_->observe(observed_at);
          }
        } catch (const std::exception & exception) {
          RCLCPP_WARN(
            get_logger(), "GetPlanningSceneState failed: %s", exception.what());
        }
      });
    if (!planning_scene_request_tracker_.bindRequestId(*token, request_handle.request_id)) {
      (void)planning_scene_client_->remove_pending_request(request_handle.request_id);
    }
  } catch (const std::exception & exception) {
    (void)planning_scene_request_tracker_.abandon(*token);
    RCLCPP_WARN(
      get_logger(), "Cannot request PlanningScene state: %s", exception.what());
  }
}

void SystemHealthMonitor::expireServiceRequests()
{
  const auto now = FreshnessMonitor::Clock::now();
  const auto expire_request =
    [this, now](
    detail::ServiceRequestTracker & tracker,
    const auto & client,
    const char * dependency)
    {
      const auto expired = tracker.expire(now);
      if (!expired) {
        return;
      }

      bool removed = false;
      if (expired->request_id) {
        removed = client->remove_pending_request(*expired->request_id);
      }
      RCLCPP_WARN(
        get_logger(),
        "%s service request exceeded its %.3f s steady deadline; "
        "pending middleware request removed=%s, polling will retry",
        dependency,
        std::chrono::duration<double>(service_request_timeout_).count(),
        removed ? "true" : "false");
    };

  expire_request(
    controller_request_tracker_, controller_client_, "Controller manager");
  expire_request(
    planning_scene_request_tracker_, planning_scene_client_,
    "PlanningScene coordinator");
}

}  // namespace manipulation::monitoring
