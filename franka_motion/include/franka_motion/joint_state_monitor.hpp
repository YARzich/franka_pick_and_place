// Responsibility: Tracks fresh Panda finger feedback and provides bounded waits for target width.

#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "franka_motion/gripper_utils.hpp"
#include "franka_motion/panda_joint_state_utils.hpp"

namespace franka_motion
{
/// Latest finite positions of both Panda finger joints, in metres.
struct GripperJointState
{
  double left{std::numeric_limits<double>::quiet_NaN()};
  double right{std::numeric_limits<double>::quiet_NaN()};
};

/// Thread-safe receipt-time monitor for JointState and Panda finger positions.
///
/// The monitor owns copied numeric state only, not a ROS subscription or
/// MoveIt state monitor. Every public operation may run concurrently.
/// Freshness uses std::chrono::steady_clock deliberately: ROS simulation time
/// may pause or jump and must not make stale hardware feedback appear fresh.
class FrankaJointStateMonitor
{
public:
  /// Record receipt of a JointState and update fingers when both are valid.
  void update(const sensor_msgs::msg::JointState & msg)
  {
    const auto finger_sample = parsePandaFingerJointState(msg);
    std::lock_guard<std::mutex> lock(mutex_);
    joint_state_seen_ = true;
    last_joint_state_wall_ = std::chrono::steady_clock::now();
    if (finger_sample.has_value()) {
      gripper_.left = finger_sample->left_position;
      gripper_.right = finger_sample->right_position;
      gripper_state_seen_ = true;
      last_gripper_state_wall_ = last_joint_state_wall_;
    }
  }

  /// Return whether any JointState was received no more than max_age ago.
  bool hasFreshJointState(std::chrono::steady_clock::duration max_age) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_state_seen_ &&
           std::chrono::steady_clock::now() - last_joint_state_wall_ <= max_age;
  }

  /// Return the last complete finite finger state regardless of age.
  std::optional<GripperJointState> gripperState() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!std::isfinite(gripper_.left) || !std::isfinite(gripper_.right)) {
      return std::nullopt;
    }
    return gripper_;
  }

  /// Return the last complete finger state only when it is at most max_age old.
  std::optional<GripperJointState> freshGripperState(
    std::chrono::steady_clock::duration max_age) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gripper_state_seen_ ||
      std::chrono::steady_clock::now() - last_gripper_state_wall_ > max_age ||
      !std::isfinite(gripper_.left) || !std::isfinite(gripper_.right))
    {
      return std::nullopt;
    }
    return gripper_;
  }

  /// Poll fresh finger states until both joints reach the target or timeout.
  ///
  /// Target/tolerance/progress values are full-width metres and the deadline
  /// uses the steady clock. Invalid/missing samples never satisfy the wait.
  /// A closing contact stall may count as success only after measured progress
  /// from command_start_width. The method blocks the calling thread, not a ROS
  /// callback that must service the JointState subscription.
  template<typename Rep, typename Period>
  bool waitForGripperWidth(
    double target_width,
    std::chrono::duration<Rep, Period> timeout,
    double tolerance,
    bool allow_stall_on_close,
    const std::string & label,
    const rclcpp::Logger & logger,
    std::chrono::steady_clock::duration max_state_age =
    std::chrono::milliseconds(500),
    std::optional<double> command_start_width = std::nullopt,
    double stall_progress_threshold = 0.0005,
    const std::function<bool()> & stop_requested = {}) const
  {
    const double target_joint = widthToJoint(target_width);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && (!stop_requested || !stop_requested()) &&
      std::chrono::steady_clock::now() < deadline)
    {
      const auto state = freshGripperState(max_state_age);
      if (state.has_value() &&
        std::abs(state->left - target_joint) <= tolerance &&
        std::abs(state->right - target_joint) <= tolerance)
      {
        return true;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    if (stop_requested && stop_requested()) {
      return false;
    }

    if (allow_stall_on_close) {
      const auto state = freshGripperState(max_state_age);
      if (state.has_value()) {
        const double width = state->left + state->right;
        if (command_start_width.has_value() && gripperStallHasClosingProgress(
            command_start_width.value(), width, target_width,
            stall_progress_threshold, 2.0 * tolerance))
        {
          RCLCPP_INFO(
            logger,
            "Хвататель остановился при закрытии "
            "на свежей ширине "
            "%.3f м, продолжаем",
            width);
          return true;
        }
      }
    }

    const auto timeout_sec = std::chrono::duration<double>(timeout).count();
    RCLCPP_WARN(
      logger,
      "Хвататель не достиг позиции %s за %.2f c",
      label.c_str(),
      timeout_sec);
    return false;
  }

private:
  mutable std::mutex mutex_;
  GripperJointState gripper_;
  bool joint_state_seen_{false};
  bool gripper_state_seen_{false};
  std::chrono::steady_clock::time_point last_joint_state_wall_{};
  std::chrono::steady_clock::time_point last_gripper_state_wall_{};
};
}  // namespace franka_motion
