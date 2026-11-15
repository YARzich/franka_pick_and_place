// Responsibility: Provides side-effect-free pose validation, correlation, and rigid geometry.

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace franka_motion
{
/// Decision for a newly received stamped pose relative to the preceding plan.
enum class CorrelatedPoseAction
{
  kIgnore,
  kRepublish,
  kReplan,
};

/// Return whether a pose carries a canonical relative frame and valid non-zero ROS stamp.
inline bool hasCorrelationHeader(const geometry_msgs::msg::PoseStamped & pose)
{
  return manipulation::validation::is_canonical_relative_frame_id(
    pose.header.frame_id) && pose.header.stamp.sec >= 0 &&
         pose.header.stamp.nanosec < 1000000000U &&
         (pose.header.stamp.sec != 0 || pose.header.stamp.nanosec != 0U);
}

/// Return whether two poses identify the exact same frame/stamp sensor sample.
inline bool hasSameCorrelationHeader(
  const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs)
{
  return hasCorrelationHeader(lhs) && hasCorrelationHeader(rhs) &&
         lhs.header.frame_id == rhs.header.frame_id &&
         lhs.header.stamp == rhs.header.stamp;
}

/// Return a numerically stable unit quaternion or `std::nullopt` for invalid input.
inline std::optional<geometry_msgs::msg::Quaternion> normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & quaternion,
  double minimum_norm = 1.0e-9)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w) ||
    !std::isfinite(minimum_norm) || minimum_norm < 0.0)
  {
    return std::nullopt;
  }

  const double scale = std::max(
    {std::abs(quaternion.x), std::abs(quaternion.y),
      std::abs(quaternion.z), std::abs(quaternion.w)});
  if (scale == 0.0) {
    return std::nullopt;
  }

  geometry_msgs::msg::Quaternion normalized;
  normalized.x = quaternion.x / scale;
  normalized.y = quaternion.y / scale;
  normalized.z = quaternion.z / scale;
  normalized.w = quaternion.w / scale;
  const double scaled_norm = std::sqrt(
    normalized.x * normalized.x + normalized.y * normalized.y +
    normalized.z * normalized.z + normalized.w * normalized.w);
  if (scale < minimum_norm / scaled_norm) {
    return std::nullopt;
  }

  normalized.x /= scaled_norm;
  normalized.y /= scaled_norm;
  normalized.z /= scaled_norm;
  normalized.w /= scaled_norm;
  return normalized;
}

/// Validate finite metre translation and a normalizable orientation.
inline bool isValidPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) &&
         normalizedQuaternion(pose.orientation, 1.0e-6).has_value();
}

/**
 * Normalize a correlated pose while optionally filling a missing frame ID.
 *
 * A valid non-zero ROS stamp remains mandatory. The returned stamp and
 * translation are unchanged; invalid pose or correlation data returns nullopt.
 */
inline std::optional<geometry_msgs::msg::PoseStamped> normalizeCorrelatedPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & fallback_frame = "")
{
  auto normalized = pose;
  if (normalized.header.frame_id.empty()) {
    normalized.header.frame_id = fallback_frame;
  }
  if (!hasCorrelationHeader(normalized) ||
    !std::isfinite(normalized.pose.position.x) ||
    !std::isfinite(normalized.pose.position.y) ||
    !std::isfinite(normalized.pose.position.z))
  {
    return std::nullopt;
  }

  const auto orientation = normalizedQuaternion(normalized.pose.orientation);
  if (!orientation.has_value()) {
    return std::nullopt;
  }
  normalized.pose.orientation = orientation.value();
  return normalized;
}

/**
 * Choose whether changed perception data requires republish or replanning.
 *
 * Translation thresholds use metres and rotation thresholds use radians.
 * Different frames always require replanning; this pure function performs no TF
 * lookup and expects both poses already expressed in comparable coordinates.
 */
inline CorrelatedPoseAction chooseCorrelatedPoseAction(
  const geometry_msgs::msg::PoseStamped & current,
  const geometry_msgs::msg::PoseStamped & previous,
  bool previous_plan_succeeded,
  bool retry_due,
  double translation_threshold_m,
  double rotation_threshold_rad)
{
  if (current.header.frame_id != previous.header.frame_id) {
    return CorrelatedPoseAction::kReplan;
  }

  const double dx = current.pose.position.x - previous.pose.position.x;
  const double dy = current.pose.position.y - previous.pose.position.y;
  const double dz = current.pose.position.z - previous.pose.position.z;
  const double translation_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  const auto current_q = normalizedQuaternion(current.pose.orientation).value_or(identity);
  const auto previous_q = normalizedQuaternion(previous.pose.orientation).value_or(identity);
  const double quaternion_dot = std::abs(
    current_q.x * previous_q.x + current_q.y * previous_q.y +
    current_q.z * previous_q.z + current_q.w * previous_q.w);
  const double rotation_delta = 2.0 * std::acos(std::clamp(quaternion_dot, 0.0, 1.0));

  if (translation_delta >= translation_threshold_m ||
    rotation_delta >= rotation_threshold_rad ||
    (!previous_plan_succeeded && retry_due))
  {
    return CorrelatedPoseAction::kReplan;
  }

  if (previous_plan_succeeded &&
    hasCorrelationHeader(current) &&
    !hasSameCorrelationHeader(current, previous))
  {
    return CorrelatedPoseAction::kRepublish;
  }
  return CorrelatedPoseAction::kIgnore;
}

/// Construct a pose from metre translation and REP-103 fixed-axis RPY radians.
inline geometry_msgs::msg::Pose makePose(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

/// Convert a pose into a rigid transform, substituting identity for invalid rotation.
inline tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  const auto orientation = normalizedQuaternion(pose.orientation).value_or(identity);
  const tf2::Quaternion rotation(
    orientation.x, orientation.y, orientation.z, orientation.w);

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  transform.setRotation(rotation);
  return transform;
}

/// Convert a rigid transform to a pose with normalized orientation.
inline geometry_msgs::msg::Pose transformToPose(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();

  tf2::Quaternion rotation = transform.getRotation();
  rotation.normalize();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}
}  // namespace franka_motion
