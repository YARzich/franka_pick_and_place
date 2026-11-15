// Responsibility: Stores the latest stamped detection and supports interruptible bounded waits.

#ifndef WORKFLOW_V2__DETECTED_POSE_BUFFER_HPP_
#define WORKFLOW_V2__DETECTED_POSE_BUFFER_HPP_

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace franka_motion
{

/**
 * Thread-safe cache for the latest detected object pose.
 *
 * The buffer owns a copied message, but no ROS subscription or TF resource.
 * Receipt freshness and wait deadlines use `std::chrono::steady_clock`.
 * Every update/wait operation may run concurrently. Destruction requires all
 * blocked waiters to have returned.
 */
class DetectedPoseBuffer
{
public:
  using SteadyClock = std::chrono::steady_clock;
  using KeepWaiting = std::function<bool()>;
  using AcceptCubePose =
    std::function<bool(const geometry_msgs::msg::PoseStamped &)>;

  /// Replace the latest cube sample and record its local steady receipt time.
  void updateCubePose(
    const geometry_msgs::msg::PoseStamped & pose,
    SteadyClock::time_point received_at = SteadyClock::now())
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_cube_pose_ = pose;
      last_cube_pose_wall_ = received_at;
    }
    changed_.notify_all();
  }

  /// Wake blocked adapters after goal cancellation or process shutdown.
  void notifyWaiters() noexcept
  {
    changed_.notify_all();
  }

  /**
   * Wait for a cube received no earlier than `not_before`.
   *
   * `accept_pose` lets the caller ignore a repeated sensor stamp without
   * removing it from the cache. `keep_waiting` makes ROS shutdown or process
   * failure observable without coupling this cache to ROS. Both callbacks run
   * while the cache mutex is held and must be short, non-blocking, and
   * non-reentrant. Returns `std::nullopt` on stop or steady-clock timeout.
   */
  template<typename Rep, typename Period>
  std::optional<geometry_msgs::msg::PoseStamped> waitForFreshCubePose(
    std::chrono::duration<Rep, Period> timeout,
    SteadyClock::time_point not_before,
    const KeepWaiting & keep_waiting,
    const AcceptCubePose & accept_pose)
  {
    const auto deadline = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(timeout);
    std::unique_lock<std::mutex> lock(mutex_);
    while (keep_waiting()) {
      if (latest_cube_pose_.has_value() && last_cube_pose_wall_ >= not_before &&
        accept_pose(*latest_cube_pose_))
      {
        return latest_cube_pose_;
      }
      if (!waitForChange(lock, deadline)) {
        break;
      }
    }
    return std::nullopt;
  }

  /// Wait for any cube sample satisfying the receipt-time freshness boundary.
  template<typename Rep, typename Period>
  std::optional<geometry_msgs::msg::PoseStamped> waitForFreshCubePose(
    std::chrono::duration<Rep, Period> timeout,
    SteadyClock::time_point not_before,
    const KeepWaiting & keep_waiting)
  {
    return waitForFreshCubePose(
      timeout, not_before, keep_waiting,
      [](const geometry_msgs::msg::PoseStamped &) {return true;});
  }

private:
  bool waitForChange(
    std::unique_lock<std::mutex> & lock,
    SteadyClock::time_point deadline)
  {
    const auto now = SteadyClock::now();
    if (now >= deadline) {
      return false;
    }

    // Periodic wake-up lets callers observe rclcpp shutdown even when no message arrives.
    const auto next_check = std::min(deadline, now + std::chrono::milliseconds(100));
    changed_.wait_until(lock, next_check);
    return SteadyClock::now() < deadline;
  }

  std::mutex mutex_;
  std::condition_variable changed_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_cube_pose_;
  SteadyClock::time_point last_cube_pose_wall_{};
};

}  // namespace franka_motion

#endif  // WORKFLOW_V2__DETECTED_POSE_BUFFER_HPP_
