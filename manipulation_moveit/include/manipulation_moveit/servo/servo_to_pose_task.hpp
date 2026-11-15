// Responsibility: Defines the ROS-transport-free ServoToPose task loop and its control state.

#ifndef MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_TASK_HPP_
#define MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_TASK_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "manipulation/servo/servo_to_pose_types.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation_moveit/servo/servo_to_pose_port.hpp"

namespace manipulation::moveit
{

/** Immutable timing and completion policy for reactive pose tracking. */
struct ServoToPoseTaskConfig
{
  using Duration = std::chrono::steady_clock::duration;

  manipulation::servo::ServoToPoseStateMachineConfig state_machine;
  Duration control_period{std::chrono::milliseconds{20}};
  Duration incoming_command_timeout{std::chrono::milliseconds{100}};
  Duration service_call_timeout{std::chrono::milliseconds{500}};
  Duration target_prepare_timeout{std::chrono::seconds{1}};
  Duration feedback_startup_timeout{std::chrono::milliseconds{500}};
  Duration observation_timeout{std::chrono::milliseconds{5}};
};

/// Return no value only when every duration and state-machine policy is usable.
[[nodiscard]] std::optional<std::string> validate_servo_to_pose_task_config(
  const ServoToPoseTaskConfig & config);

/// The total goal timeout must leave at least one control cycle before halt reserve.
[[nodiscard]] bool servo_to_pose_timeout_has_operational_budget(
  ServoToPoseTaskConfig::Duration total_timeout,
  const ServoToPoseTaskConfig & config) noexcept;

/** Thread-safe pause/cancel intent and interruptible control-loop wait. */
class ServoToPoseTaskControl final
{
public:
  struct Snapshot
  {
    bool cancel_requested{false};
    bool pause_requested{false};
    std::uint64_t revision{0U};
  };

  void request_cancel();
  void request_pause();
  void resume();

  [[nodiscard]] Snapshot snapshot() const;
  void wait_until(std::chrono::steady_clock::time_point deadline, std::uint64_t revision);

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool cancel_requested_{false};
  bool pause_requested_{false};
  std::uint64_t revision_{0U};
};

/** One feedback sample produced by the task loop before ROS serialization. */
struct ServoToPoseTaskFeedback
{
  manipulation::task::TaskStateSnapshot state;
  bool tracking_error_valid{false};
  double linear_error_m{0.0};
  double angular_error_rad{0.0};
};

/** Motion-only terminal result preserving safe-stop evidence. */
struct ServoToPoseTaskResult
{
  manipulation::task::TaskErrorCode error_code{
    manipulation::task::TaskErrorCode::kInternalError};
  bool stop_confirmed{false};
  std::chrono::steady_clock::duration elapsed{};
  std::string detail;

  [[nodiscard]] bool succeeded() const noexcept
  {
    return error_code == manipulation::task::TaskErrorCode::kNone;
  }
};

/**
 * Synchronous task driver for one already-admitted ServoToPose goal.
 *
 * The caller owns the worker thread and the resource lease. This class owns no
 * ROS entities and performs no resource admission. It prepares a fixed target,
 * regularly republishes it, and routes every normal or exceptional stop
 * through ServoToPoseStateMachine before returning.
 */
class ServoToPoseTask final
{
public:
  using Clock = std::chrono::steady_clock;
  using FeedbackCallback = std::function<void(const ServoToPoseTaskFeedback &)>;

  ServoToPoseTask(ServoToPosePort & port, ServoToPoseTaskConfig config);

  [[nodiscard]] ServoToPoseTaskResult run(
    const geometry_msgs::msg::PoseStamped & target,
    Clock::time_point started_at,
    Clock::time_point absolute_deadline,
    ServoToPoseTaskControl & control,
    FeedbackCallback feedback_callback);

  /**
   * Last-resort stop path for an execution host that catches an escaped task failure.
   *
   * This method never resumes or publishes a target. It only observes fresh
   * velocity evidence, requests backend pause when required, and returns after
   * stop confirmation or the existing absolute deadline.
   */
  [[nodiscard]] ServoToPoseTaskResult halt_after_unexpected_failure(
    Clock::time_point started_at,
    Clock::time_point absolute_deadline,
    std::string detail) noexcept;

  [[nodiscard]] const ServoToPoseTaskConfig & config() const noexcept;

private:
  [[nodiscard]] ServoToPoseTaskResult run_impl(
    const geometry_msgs::msg::PoseStamped & target,
    Clock::time_point started_at,
    Clock::time_point absolute_deadline,
    ServoToPoseTaskControl & control,
    const FeedbackCallback & feedback_callback);
  [[nodiscard]] ServoToPoseTaskResult halt_after_failure(
    Clock::time_point started_at,
    Clock::time_point absolute_deadline,
    manipulation::task::TaskErrorCode error_code,
    std::string detail) noexcept;

  ServoToPosePort & port_;
  const ServoToPoseTaskConfig config_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_TASK_HPP_
