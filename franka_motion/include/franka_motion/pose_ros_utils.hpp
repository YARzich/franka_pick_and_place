// Responsibility: Adapts pose geometry to ROS time, logging, and exact-stamp TF operations.

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include "franka_motion/pose_geometry.hpp"

namespace franka_motion
{
/**
 * Build a local transform from XYZ metres and REP-103 RPY radians.
 *
 * Invalid vector lengths or non-finite values are logged and return `fallback`.
 */
inline tf2::Transform makeLocalTransform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy,
  const tf2::Transform & fallback,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  const auto contains_only_finite_values = [](const std::vector<double> & values) {
      return std::all_of(values.begin(), values.end(), [](double value) {
                 return std::isfinite(value);
        });
    };
  if (xyz.size() != 3 || rpy.size() != 3 ||
    !contains_only_finite_values(xyz) || !contains_only_finite_values(rpy))
  {
    RCLCPP_WARN(logger, "%s должен содержать конечные xyz/rpy по 3 значения, использую fallback",
      label.c_str());
    return fallback;
  }

  tf2::Quaternion rotation;
  rotation.setRPY(rpy[0], rpy[1], rpy[2]);
  rotation.normalize();

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
  transform.setRotation(rotation);
  return transform;
}

/// Stamp a pose in an explicitly supplied frame without transforming it.
inline geometry_msgs::msg::PoseStamped makePoseStamped(
  const geometry_msgs::msg::Pose & pose,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.frame_id = frame_id;
  stamped.header.stamp = stamp;
  stamped.pose = pose;
  return stamped;
}

/// Apply a metre translation in the pose's current frame and replace its stamp.
inline geometry_msgs::msg::PoseStamped translatedPose(
  geometry_msgs::msg::PoseStamped pose,
  double dx,
  double dy,
  double dz,
  const rclcpp::Time & stamp)
{
  pose.header.stamp = stamp;
  pose.pose.position.x += dx;
  pose.pose.position.y += dy;
  pose.pose.position.z += dz;
  return pose;
}

/**
 * Transform a correlated pose at its exact measurement stamp.
 *
 * This returns `T_target_source * pose_source` in `target_frame`. The source
 * frame may use `fallback_source_frame` only when the message frame is empty;
 * latest-TF fallback is intentionally forbidden because it would mislabel the
 * sensor sample. Translation is metres, timeout is seconds, and TF/validation
 * failure returns `std::nullopt` after logging.
 */
inline std::optional<geometry_msgs::msg::PoseStamped> transformPoseToFrame(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame,
  const std::string & fallback_source_frame,
  const tf2_ros::Buffer & tf_buffer,
  const rclcpp::Logger & logger,
  double timeout_sec)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(target_frame)) {
    RCLCPP_ERROR(
      logger,
      "target_frame для TF-преобразования должен быть canonical relative frame");
    return std::nullopt;
  }
  if (!isValidPose(pose.pose)) {
    RCLCPP_ERROR(logger, "PoseStamped содержит некорректную позицию или quaternion");
    return std::nullopt;
  }

  auto source_pose = pose;
  source_pose.pose.orientation = normalizedQuaternion(source_pose.pose.orientation).value();
  if (source_pose.header.frame_id.empty()) {
    if (!manipulation::validation::is_canonical_relative_frame_id(
        fallback_source_frame))
    {
      RCLCPP_ERROR(
        logger, "Пустой PoseStamped требует canonical fallback source frame");
      return std::nullopt;
    }
    source_pose.header.frame_id = fallback_source_frame;
    RCLCPP_WARN(logger, "PoseStamped пришел без frame_id, использую %s",
      fallback_source_frame.c_str());
  }
  if (!manipulation::validation::is_canonical_relative_frame_id(
      source_pose.header.frame_id))
  {
    RCLCPP_ERROR(logger, "PoseStamped содержит некорректный source frame_id");
    return std::nullopt;
  }
  if (!hasCorrelationHeader(source_pose)) {
    RCLCPP_ERROR(logger, "TF-преобразование требует валидный ненулевой source stamp");
    return std::nullopt;
  }

  if (source_pose.header.frame_id == target_frame) {
    return source_pose;
  }

  try {
    const auto transform = tf_buffer.lookupTransform(
      target_frame,
      source_pose.header.frame_id,
      tf2_ros::fromMsg(source_pose.header.stamp),
      tf2::durationFromSec(timeout_sec));
    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(source_pose, transformed, transform);
    transformed.header.frame_id = target_frame;
    transformed.header.stamp = source_pose.header.stamp;
    if (!isValidPose(transformed.pose)) {
      RCLCPP_ERROR(logger, "TF-преобразование вернуло некорректную pose");
      return std::nullopt;
    }
    return transformed;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger,
      "Не удалось преобразовать pose из %s в %s: %s",
      source_pose.header.frame_id.c_str(),
      target_frame.c_str(),
      ex.what());
    return std::nullopt;
  }
}
}  // namespace franka_motion
