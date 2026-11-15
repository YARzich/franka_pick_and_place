// Responsibility: Performs deadline-bounded MoveIt Servo mode and pause service calls.

#include "moveit_servo_ros_adapter_impl.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace manipulation::moveit
{
namespace
{

template<typename ClientT>
bool wait_until_ready(
  const std::shared_ptr<ClientT> & client,
  const MoveItServoRosAdapter::Deadline deadline,
  const std::chrono::steady_clock::duration poll_period)
{
  while (rclcpp::ok()) {
    if (client->service_is_ready()) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::min(poll_period, deadline - now));
  }
  return false;
}

template<typename ClientT, typename PendingRequestT>
bool wait_for_response(
  const std::shared_ptr<ClientT> & client,
  PendingRequestT & pending,
  const MoveItServoRosAdapter::Deadline deadline,
  const std::chrono::steady_clock::duration poll_period)
{
  while (rclcpp::ok()) {
    if (pending.wait_for(std::chrono::steady_clock::duration::zero()) ==
      std::future_status::ready)
    {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      client->remove_pending_request(pending);
      return false;
    }
    std::this_thread::sleep_for(std::min(poll_period, deadline - now));
  }
  client->remove_pending_request(pending);
  return false;
}

}  // namespace

MoveItServoRosResult MoveItServoRosAdapter::Impl::timeout_result(
  const std::string & operation)
{
  return MoveItServoRosResult::failure(
    MoveItServoRosError::kTimeout, operation + " exceeded its steady deadline");
}

MoveItServoRosResult MoveItServoRosAdapter::Impl::configure_pose_mode(
  const Deadline deadline)
{
  return call_command_type_pose(deadline);
}

MoveItServoRosResult MoveItServoRosAdapter::Impl::set_paused(
  const bool paused, const Deadline deadline)
{
  return call_pause_service(paused, deadline);
}

MoveItServoRosResult MoveItServoRosAdapter::Impl::call_command_type_pose(
  const Deadline deadline)
{
  if (!service_mutex_.try_lock_until(deadline)) {
    return timeout_result("waiting for exclusive Servo service access");
  }
  const std::unique_lock<std::timed_mutex> service_lock(service_mutex_, std::adopt_lock);

  if (!wait_until_ready(command_type_client_, deadline, config_.poll_period)) {
    return timeout_result("waiting for ServoCommandType service");
  }

  auto request = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
  try {
    auto pending = command_type_client_->async_send_request(std::move(request));
    if (!wait_for_response(
        command_type_client_, pending, deadline, config_.poll_period))
    {
      return timeout_result("switching MoveIt Servo to POSE mode");
    }
    const auto response = pending.get();
    if (!response || !response->success) {
      return MoveItServoRosResult::failure(
        MoveItServoRosError::kRejected,
        "MoveIt Servo rejected POSE command mode");
    }
  } catch (const std::exception & error) {
    return MoveItServoRosResult::failure(
      MoveItServoRosError::kUnavailable,
      std::string{"ServoCommandType call failed: "} + error.what());
  }
  return MoveItServoRosResult::success("MoveIt Servo POSE mode confirmed");
}

MoveItServoRosResult MoveItServoRosAdapter::Impl::call_pause_service(
  const bool paused, const Deadline deadline)
{
  if (!service_mutex_.try_lock_until(deadline)) {
    return timeout_result("waiting for exclusive Servo service access");
  }
  const std::unique_lock<std::timed_mutex> service_lock(service_mutex_, std::adopt_lock);

  if (!wait_until_ready(pause_client_, deadline, config_.poll_period)) {
    const std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    backend_state_ = manipulation::servo::ServoBackendState::kUnknown;
    return timeout_result("waiting for Servo pause service");
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = paused;
  try {
    auto pending = pause_client_->async_send_request(std::move(request));
    if (!wait_for_response(pause_client_, pending, deadline, config_.poll_period)) {
      const std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      backend_state_ = manipulation::servo::ServoBackendState::kUnknown;
      return timeout_result(paused ? "pausing MoveIt Servo" : "unpausing MoveIt Servo");
    }
    const auto response = pending.get();
    if (!response || !response->success) {
      const std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
      backend_state_ = manipulation::servo::ServoBackendState::kUnknown;
      return MoveItServoRosResult::failure(
        MoveItServoRosError::kRejected,
        paused ? "MoveIt Servo rejected pause" : "MoveIt Servo rejected unpause");
    }
  } catch (const std::exception & error) {
    const std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    backend_state_ = manipulation::servo::ServoBackendState::kUnknown;
    return MoveItServoRosResult::failure(
      MoveItServoRosError::kUnavailable,
      std::string{"Servo pause service failed: "} + error.what());
  }

  {
    const std::lock_guard<std::mutex> feedback_lock(feedback_mutex_);
    backend_state_ = paused ? manipulation::servo::ServoBackendState::kPaused :
      manipulation::servo::ServoBackendState::kRunning;
  }
  return MoveItServoRosResult::success(
    paused ? "MoveIt Servo pause confirmed" : "MoveIt Servo resume confirmed");
}

}  // namespace manipulation::moveit
