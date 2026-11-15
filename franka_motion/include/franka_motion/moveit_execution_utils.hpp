// Responsibility: Provides bounded MoveIt plan/execute retries for pose, Cartesian and joint goals.

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "franka_motion/joint_state_monitor.hpp"

namespace franka_motion
{
/**
 * Wait for both fresh JointState receipt and a complete MoveIt RobotState.
 *
 * Freshness and the outer deadline use `std::chrono::steady_clock`, so paused
 * or reset simulation time cannot revive stale feedback. This function blocks
 * its caller and requires another executor thread to service JointState and
 * MoveIt state-monitor callbacks.
 */
template<typename MoveGroupT>
bool waitForCurrentState(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const std::string & stage,
  std::chrono::steady_clock::duration timeout,
  std::chrono::steady_clock::duration max_age = std::chrono::milliseconds(500),
  const std::function<bool()> & stop_requested = {})
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok() && (!stop_requested || !stop_requested()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    if (!joint_state_monitor.hasFreshJointState(max_age)) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    auto state = move_group.getCurrentState(0.5);
    if (state) {
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_WARN(logger, "Нет актуального RobotState на этапе %s", stage.c_str());
  return false;
}

namespace detail
{
inline bool stopRequested(const std::function<bool()> & stop_requested)
{
  return stop_requested && stop_requested();
}

enum class PlanAttemptResult
{
  kReadyToExecute,
  kRetry,
  kAbort,
};

struct PlanAttempt
{
  PlanAttemptResult result;
  double completion{1.0};
};

struct PoseLogPolicy
{
  static constexpr const char * planning_retry = "Планирование %s не удалось, повтор";
  static constexpr const char * planning_failure = "Планирование %s не удалось";
  static constexpr const char * execution_retry = "Выполнение %s не удалось, повтор";
  static constexpr const char * execution_failure = "Выполнение %s не удалось";
  static constexpr const char * success = "Достигнута %s";
  static constexpr bool planning_failure_is_error = true;
  static constexpr bool execution_failure_is_error = true;
  static constexpr bool reports_completion = false;
};

struct CartesianLogPolicy
{
  static constexpr const char * planning_retry = "Cartesian %s на %.0f%%, повтор";
  static constexpr const char * planning_failure = "Cartesian %s выполнен на %.0f%%";
  static constexpr const char * execution_retry = "Выполнение %s не удалось, повтор";
  static constexpr const char * execution_failure = "Выполнение %s не удалось";
  static constexpr const char * success = "Достигнута %s";
  static constexpr bool planning_failure_is_error = false;
  static constexpr bool execution_failure_is_error = false;
  static constexpr bool reports_completion = true;
};

struct JointsLogPolicy
{
  static constexpr const char * planning_retry = "Не удалось распланировать %s, повтор";
  static constexpr const char * planning_failure = "Не удалось распланировать %s";
  static constexpr const char * execution_retry = "Выполнение %s не удалось, повтор";
  static constexpr const char * execution_failure = "Выполнение %s не удалось";
  static constexpr const char * success = "Достигнуто %s";
  static constexpr bool planning_failure_is_error = false;
  static constexpr bool execution_failure_is_error = false;
  static constexpr bool reports_completion = false;
};

inline void logRetryableFailure(
  const rclcpp::Logger & logger,
  const std::string & tag,
  bool will_retry,
  const char * retry_message,
  const char * failure_message,
  bool failure_is_error)
{
  if (will_retry) {
    RCLCPP_WARN(logger, retry_message, tag.c_str());
  } else if (failure_is_error) {
    RCLCPP_ERROR(logger, failure_message, tag.c_str());
  } else {
    RCLCPP_WARN(logger, failure_message, tag.c_str());
  }
}

template<typename LogPolicyT>
void logPlanningFailure(
  const rclcpp::Logger & logger,
  const std::string & tag,
  bool will_retry,
  double completion)
{
  if constexpr (LogPolicyT::reports_completion) {
    if (will_retry) {
      RCLCPP_WARN(logger, LogPolicyT::planning_retry, tag.c_str(), completion * 100.0);
    } else {
      RCLCPP_WARN(logger, LogPolicyT::planning_failure, tag.c_str(), completion * 100.0);
    }
  } else {
    logRetryableFailure(
      logger,
      tag,
      will_retry,
      LogPolicyT::planning_retry,
      LogPolicyT::planning_failure,
      LogPolicyT::planning_failure_is_error);
  }
}

// Runs the common bounded fresh-state -> plan -> execute lifecycle. Target construction and
// scenario-specific logging stay in callbacks so the public wrappers preserve their behavior.
template<
  typename MoveGroupT,
  typename ExecutableT,
  typename LogPolicyT,
  typename PrepareExecutableT>
bool planAndExecuteWithRetry(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const std::string & tag,
  int max_attempts,
  std::chrono::steady_clock::duration max_state_age,
  std::chrono::steady_clock::duration current_state_timeout,
  const std::function<bool()> & stop_requested,
  PrepareExecutableT && prepare_executable)
{
  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    if (stopRequested(stop_requested)) {
      return false;
    }
    if (!waitForCurrentState(
        move_group, joint_state_monitor, logger, tag, current_state_timeout, max_state_age,
        stop_requested))
    {
      return false;
    }
    if (stopRequested(stop_requested)) {
      return false;
    }
    move_group.setStartStateToCurrentState();

    ExecutableT executable;
    const bool will_retry = attempt < max_attempts;
    const PlanAttempt planning = prepare_executable(executable);
    if (stopRequested(stop_requested)) {
      return false;
    }
    if (planning.result == PlanAttemptResult::kAbort) {
      return false;
    }
    if (planning.result == PlanAttemptResult::kRetry) {
      logPlanningFailure<LogPolicyT>(logger, tag, will_retry, planning.completion);
      if (will_retry) {
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      return false;
    }

    std::atomic<bool> execution_finished{false};
    std::thread stop_monitor;
    if (stop_requested) {
      stop_monitor = std::thread([&move_group, &stop_requested, &execution_finished]() {
            while (!execution_finished.load(std::memory_order_acquire)) {
              if (stopRequested(stop_requested)) {
                move_group.stop();
                return;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    const auto finish_stop_monitor = [&execution_finished, &stop_monitor]() {
        execution_finished.store(true, std::memory_order_release);
        if (stop_monitor.joinable()) {
          stop_monitor.join();
        }
      };
    const auto execution_result = [&]() {
        try {
          const auto result = move_group.execute(executable);
          finish_stop_monitor();
          return result;
        } catch (...) {
          finish_stop_monitor();
          throw;
        }
      }();
    move_group.stop();
    if (stopRequested(stop_requested)) {
      return false;
    }
    if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, LogPolicyT::success, tag.c_str());
      return true;
    }

    logRetryableFailure(
      logger,
      tag,
      will_retry,
      LogPolicyT::execution_retry,
      LogPolicyT::execution_failure,
      LogPolicyT::execution_failure_is_error);
    if (will_retry) {
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    return false;
  }
  return false;
}
}  // namespace detail

/**
 * Plan and execute an unstamped pose in MoveGroup's configured pose frame.
 *
 * Each retry starts from fresh measured state and clears the pose target after
 * planning. `current_state_timeout` bounds each pre-plan wait with the steady
 * clock. A false result may follow partial robot motion; callers must inspect
 * measured state before recovery rather than assuming the start pose remains.
 */
template<typename MoveGroupT>
inline bool planAndExecutePose(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & tag,
  int max_attempts = 2,
  std::chrono::steady_clock::duration max_state_age = std::chrono::milliseconds(500),
  const std::function<bool()> & stop_requested = {},
  std::chrono::steady_clock::duration current_state_timeout = std::chrono::seconds(3))
{
  return detail::planAndExecuteWithRetry<
    MoveGroupT, typename MoveGroupT::Plan, detail::PoseLogPolicy>(
    move_group,
    joint_state_monitor,
    logger,
    tag,
    max_attempts,
    max_state_age,
    current_state_timeout,
    stop_requested,
    [&move_group, &target_pose](auto & plan) {
      move_group.setPoseTarget(target_pose);
      const auto result = move_group.plan(plan);
      move_group.clearPoseTargets();
      return detail::PlanAttempt{
        result == moveit::core::MoveItErrorCode::SUCCESS ?
        detail::PlanAttemptResult::kReadyToExecute : detail::PlanAttemptResult::kRetry};
    });
}

/**
 * Plan and execute a stamped end-effector pose in its declared frame.
 *
 * The pose must carry the frame expected by MoveIt and `end_effector_link` must
 * name the controlled link. Retries use fresh state; false does not guarantee
 * that execution left the robot unchanged.
 */
template<typename MoveGroupT>
inline bool planAndExecutePose(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::PoseStamped & target_pose,
  const std::string & end_effector_link,
  const std::string & tag,
  int max_attempts = 2,
  std::chrono::steady_clock::duration max_state_age = std::chrono::milliseconds(500),
  const std::function<bool()> & stop_requested = {},
  std::chrono::steady_clock::duration current_state_timeout = std::chrono::seconds(3))
{
  return detail::planAndExecuteWithRetry<
    MoveGroupT, typename MoveGroupT::Plan, detail::PoseLogPolicy>(
    move_group,
    joint_state_monitor,
    logger,
    tag,
    max_attempts,
    max_state_age,
    current_state_timeout,
    stop_requested,
    [&move_group, &target_pose, &end_effector_link, &logger, &tag](auto & plan)
    {
      if (!move_group.setPoseTarget(target_pose, end_effector_link)) {
        RCLCPP_ERROR(logger, "Некорректная pose target для %s", tag.c_str());
        move_group.clearPoseTargets();
        return detail::PlanAttempt{detail::PlanAttemptResult::kAbort};
      }
      const auto result = move_group.plan(plan);
      move_group.clearPoseTargets();
      return detail::PlanAttempt{
        result == moveit::core::MoveItErrorCode::SUCCESS ?
        detail::PlanAttemptResult::kReadyToExecute : detail::PlanAttemptResult::kRetry};
    });
}

/**
 * Compute and execute a one-waypoint Cartesian path.
 *
 * The target uses MoveGroup's configured pose frame and units. A plan is
 * accepted only at 99 percent completion or better. Execution failure may
 * leave the robot partway along the trajectory.
 */
template<typename MoveGroupT>
inline bool planAndExecuteCartesian(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & tag,
  int max_attempts = 2,
  std::chrono::steady_clock::duration max_state_age = std::chrono::milliseconds(500),
  const std::function<bool()> & stop_requested = {},
  std::chrono::steady_clock::duration current_state_timeout = std::chrono::seconds(3))
{
  return detail::planAndExecuteWithRetry<
    MoveGroupT, moveit_msgs::msg::RobotTrajectory, detail::CartesianLogPolicy>(
    move_group,
    joint_state_monitor,
    logger,
    tag,
    max_attempts,
    max_state_age,
    current_state_timeout,
    stop_requested,
    [&move_group, &target_pose](auto & trajectory)
    {
      const std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
      const double fraction = move_group.computeCartesianPath(waypoints, 0.01, trajectory);
      return detail::PlanAttempt{
        fraction < 0.99 ?
        detail::PlanAttemptResult::kRetry : detail::PlanAttemptResult::kReadyToExecute,
        fraction};
    });
}

/**
 * Plan and execute a full joint target in the planning group's model units.
 *
 * The vector ordering must match MoveGroup's active joint model group. Retries
 * start from fresh measured state; false may still follow partial execution.
 */
template<typename MoveGroupT>
inline bool planAndExecuteJoints(
  MoveGroupT & move_group,
  const FrankaJointStateMonitor & joint_state_monitor,
  const rclcpp::Logger & logger,
  const std::vector<double> & joints,
  const std::string & tag,
  int max_attempts = 2,
  std::chrono::steady_clock::duration max_state_age = std::chrono::milliseconds(500),
  const std::function<bool()> & stop_requested = {},
  std::chrono::steady_clock::duration current_state_timeout = std::chrono::seconds(3))
{
  return detail::planAndExecuteWithRetry<
    MoveGroupT, typename MoveGroupT::Plan, detail::JointsLogPolicy>(
    move_group,
    joint_state_monitor,
    logger,
    tag,
    max_attempts,
    max_state_age,
    current_state_timeout,
    stop_requested,
    [&move_group, &joints](auto & plan) {
      move_group.setJointValueTarget(joints);
      const auto result = move_group.plan(plan);
      return detail::PlanAttempt{
        result == moveit::core::MoveItErrorCode::SUCCESS ?
        detail::PlanAttemptResult::kReadyToExecute : detail::PlanAttemptResult::kRetry};
    });
}
}  // namespace franka_motion
