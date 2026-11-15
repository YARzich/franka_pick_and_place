// Responsibility: Canonicalizes task goals and measures task error with deadline-bounded TF reads.

#include "moveit_servo_ros_adapter_impl.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <utility>

#include "tf2/time.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::moveit
{
namespace
{

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

geometry_msgs::msg::TransformStamped identity_transform(
  const std::string & target_frame,
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = target_frame;
  transform.header.stamp = stamp;
  transform.child_frame_id = source_frame;
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

MoveItServoRosAdapter::Impl::TransformLookupResult
MoveItServoRosAdapter::Impl::transform_failure(
  const MoveItServoRosError error, std::string detail)
{
  return {MoveItServoRosResult::failure(error, std::move(detail)), std::nullopt};
}

MoveItServoRosAdapter::Impl::TransformLookupResult
MoveItServoRosAdapter::Impl::lookup_transform_exact(
  const std::string & target_frame,
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp,
  const Deadline deadline)
{
  if (target_frame == source_frame) {
    return {
      MoveItServoRosResult::success("exact transform is identity"),
      identity_transform(target_frame, source_frame, stamp)};
  }

  std::string last_reason{"exact-stamp TF is not buffered"};
  while (rclcpp::ok()) {
    if (tf_buffer_.canTransform(
        target_frame, source_frame, tf2_ros::fromMsg(stamp),
        tf2::Duration::zero(), &last_reason))
    {
      try {
        return {
          MoveItServoRosResult::success("exact-stamp TF acquired"),
          tf_buffer_.lookupTransform(
            target_frame, source_frame, tf2_ros::fromMsg(stamp),
            tf2::Duration::zero())};
      } catch (const tf2::TransformException & error) {
        last_reason = error.what();
      }
    }

    const auto now = SteadyClock::now();
    if (now >= deadline) {
      return transform_failure(
        MoveItServoRosError::kTimeout,
        "exact-stamp TF deadline exceeded: " + last_reason);
    }
    std::this_thread::sleep_for(std::min(config_.poll_period, deadline - now));
  }
  return transform_failure(
    MoveItServoRosError::kUnavailable,
    "ROS shutdown while waiting for exact-stamp TF");
}

MoveItServoRosAdapter::Impl::TransformLookupResult
MoveItServoRosAdapter::Impl::lookup_transform_latest(
  const std::string & target_frame,
  const std::string & source_frame,
  const Deadline deadline)
{
  if (target_frame == source_frame) {
    const auto stamp = static_cast<builtin_interfaces::msg::Time>(node_.now());
    return {
      MoveItServoRosResult::success("latest transform is identity"),
      identity_transform(target_frame, source_frame, stamp)};
  }

  std::string last_reason{"latest TF is not buffered"};
  while (rclcpp::ok()) {
    if (tf_buffer_.canTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2::Duration::zero(), &last_reason))
    {
      try {
        return {
          MoveItServoRosResult::success("latest TF acquired"),
          tf_buffer_.lookupTransform(
            target_frame, source_frame, tf2::TimePointZero,
            tf2::Duration::zero())};
      } catch (const tf2::TransformException & error) {
        last_reason = error.what();
      }
    }

    const auto now = SteadyClock::now();
    if (now >= deadline) {
      return transform_failure(
        MoveItServoRosError::kTimeout,
        "current task TF deadline exceeded: " + last_reason);
    }
    std::this_thread::sleep_for(std::min(config_.poll_period, deadline - now));
  }
  return transform_failure(
    MoveItServoRosError::kUnavailable,
    "ROS shutdown while waiting for current task TF");
}

MoveItServoRosAdapter::Impl::PoseTransformResult
MoveItServoRosAdapter::Impl::transform_pose_exact(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame,
  const Deadline deadline)
{
  if (pose.header.frame_id == target_frame) {
    return {
      MoveItServoRosResult::success("target is already in planning frame"), pose};
  }

  const auto transform = lookup_transform_exact(
    target_frame, pose.header.frame_id, pose.header.stamp, deadline);
  if (!transform.status.succeeded() || !transform.transform) {
    return {transform.status, std::nullopt};
  }
  try {
    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(pose, transformed, *transform.transform);
    transformed.header.frame_id = target_frame;
    transformed.header.stamp = pose.header.stamp;
    if (!manipulation::validation::is_valid_pose_stamped(
        transformed, target_frame))
    {
      return {
        MoveItServoRosResult::failure(
          MoveItServoRosError::kInvalidData,
          "exact-stamp TF produced an invalid canonical task target"),
        std::nullopt};
    }
    return {
      MoveItServoRosResult::success("task target canonicalized at exact stamp"),
      std::move(transformed)};
  } catch (const tf2::TransformException & error) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kUnavailable,
        std::string{"exact-stamp pose transformation failed: "} + error.what()),
      std::nullopt};
  }
}

MoveItServoPrepareTargetResult MoveItServoRosAdapter::Impl::prepare_target(
  const geometry_msgs::msg::PoseStamped & task_goal,
  const Deadline deadline)
{
  {
    const std::lock_guard<std::mutex> lock(target_mutex_);
    prepared_target_.reset();
  }
  reset_ros_time_epoch();

  if (!manipulation::validation::is_valid_pose_stamped(task_goal)) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData,
        "Servo task target requires a canonical frame, non-zero stamp, finite pose, "
        "and unit quaternion"),
      std::nullopt};
  }

  auto canonical = transform_pose_exact(task_goal, config_.planning_frame, deadline);
  if (!canonical.status.succeeded() || !canonical.pose) {
    return {canonical.status, std::nullopt};
  }
  auto tip_to_task_lookup = lookup_transform_exact(
    config_.servo_tip_frame, config_.task_frame, task_goal.header.stamp, deadline);
  if (!tip_to_task_lookup.status.succeeded() || !tip_to_task_lookup.transform) {
    return {tip_to_task_lookup.status, std::nullopt};
  }

  MoveItServoPreparedTarget prepared;
  prepared.task_goal = *canonical.pose;
  try {
    Eigen::Isometry3d planning_to_task_goal;
    tf2::fromMsg(prepared.task_goal.pose, planning_to_task_goal);
    const Eigen::Isometry3d tip_to_task =
      tf2::transformToEigen(*tip_to_task_lookup.transform);
    const Eigen::Isometry3d planning_to_tip_goal =
      planning_to_task_goal * tip_to_task.inverse();
    if (!planning_to_tip_goal.matrix().allFinite()) {
      return {
        MoveItServoRosResult::failure(
          MoveItServoRosError::kInvalidData,
          "task-to-Servo-tip composition produced non-finite geometry"),
        std::nullopt};
    }
    prepared.servo_tip_goal.header = prepared.task_goal.header;
    prepared.servo_tip_goal.pose = tf2::toMsg(planning_to_tip_goal);
  } catch (const std::exception & error) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData,
        std::string{"task-to-Servo-tip composition failed: "} + error.what()),
      std::nullopt};
  }

  if (!manipulation::validation::is_valid_pose_stamped(
      prepared.servo_tip_goal, config_.planning_frame))
  {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData,
        "computed Servo tip target is not a valid stamped rigid pose"),
      std::nullopt};
  }

  {
    const std::lock_guard<std::mutex> lock(target_mutex_);
    prepared_target_ = prepared;
  }
  return {
    MoveItServoRosResult::success(
      "task target canonicalized and Servo tip offset applied"),
    std::move(prepared)};
}

MoveItServoObservationResult MoveItServoRosAdapter::Impl::latest_observation(
  const Deadline deadline)
{
  std::optional<MoveItServoPreparedTarget> target;
  {
    const std::lock_guard<std::mutex> lock(target_mutex_);
    target = prepared_target_;
  }

  const auto steady_now = SteadyClock::now();
  if (!target) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData,
        "cannot measure Servo error without a prepared target"),
      update_and_snapshot_feedback(std::nullopt, steady_now)};
  }

  const auto current = lookup_transform_latest(
    config_.planning_frame, config_.task_frame, deadline);
  if (!current.status.succeeded() || !current.transform) {
    return {
      current.status,
      update_and_snapshot_feedback(std::nullopt, SteadyClock::now())};
  }

  CurrentPoseMeasurement measurement;
  measurement.stamp = current.transform->header.stamp;
  const bool normalized_stamp =
    manipulation::validation::is_normalized_nonzero_stamp(measurement.stamp);
  const auto current_ros_now = node_.now();
  const std::int64_t now_ns = current_ros_now.nanoseconds();
  const std::int64_t transform_ns = stamp_nanoseconds(measurement.stamp);
  measurement.time_discontinuity = normalized_stamp && now_ns < transform_ns;
  if (normalized_stamp && !measurement.time_discontinuity) {
    const auto age = std::chrono::nanoseconds{now_ns - transform_ns};
    measurement.fresh = age <= std::chrono::duration_cast<std::chrono::nanoseconds>(
      config_.current_tf_max_age);
  }

  try {
    const Eigen::Isometry3d planning_to_current_task =
      tf2::transformToEigen(*current.transform);
    Eigen::Isometry3d planning_to_goal_task;
    tf2::fromMsg(target->task_goal.pose, planning_to_goal_task);
    const Eigen::Isometry3d current_task_to_goal_task =
      planning_to_current_task.inverse() * planning_to_goal_task;
    const Eigen::Quaterniond rotation{current_task_to_goal_task.rotation()};
    measurement.position_error_m = current_task_to_goal_task.translation().norm();
    measurement.orientation_error_rad =
      2.0 * std::atan2(rotation.vec().norm(), std::abs(rotation.w()));
    measurement.valid = current_task_to_goal_task.matrix().allFinite() &&
      std::isfinite(measurement.position_error_m) &&
      std::isfinite(measurement.orientation_error_rad) &&
      normalized_stamp;
  } catch (const std::exception &) {
    measurement.valid = false;
  }

  auto observation = update_and_snapshot_feedback(
    measurement, SteadyClock::now());
  if (!measurement.valid) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData,
        "current task TF did not produce a valid pose-error observation"),
      std::move(observation)};
  }
  return {
    MoveItServoRosResult::success("freshest available Servo observation captured"),
    std::move(observation)};
}

}  // namespace manipulation::moveit
