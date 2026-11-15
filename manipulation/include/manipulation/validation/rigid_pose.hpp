// Responsibility: Defines shared, side-effect-free validation for stamped rigid poses.

#ifndef MANIPULATION__VALIDATION__RIGID_POSE_HPP_
#define MANIPULATION__VALIDATION__RIGID_POSE_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <string_view>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "manipulation/validation/frame_id.hpp"

namespace manipulation::validation
{

/** Canonical absolute tolerance for rigid-pose validation and comparison. */
inline constexpr double kRigidPoseTolerance = 1.0e-6;

/** Named tolerances for approximate rigid-pose equality. */
struct RigidPoseComparisonTolerances
{
  double position{kRigidPoseTolerance};
  double orientation_component{kRigidPoseTolerance};
};

/** Return true when a ROS time is non-negative and nanoseconds are normalized. */
[[nodiscard]] inline bool is_normalized_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

/** Return true when a ROS time is normalized and not the zero sentinel. */
[[nodiscard]] inline bool is_normalized_nonzero_stamp(
  const builtin_interfaces::msg::Time & stamp)
{
  return is_normalized_stamp(stamp) && (stamp.sec != 0 || stamp.nanosec != 0U);
}

/** Return true when every translation and orientation component is finite. */
[[nodiscard]] inline bool has_finite_components(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

/** Return true when the finite quaternion has unit squared norm within the canonical tolerance. */
[[nodiscard]] inline bool is_unit_quaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  const double squared_norm =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(squared_norm) &&
         std::abs(squared_norm - 1.0) <= kRigidPoseTolerance;
}

/** Return true when a pose is finite and carries a unit quaternion. */
[[nodiscard]] inline bool is_valid_rigid_pose(const geometry_msgs::msg::Pose & pose)
{
  return has_finite_components(pose) && is_unit_quaternion(pose.orientation);
}

/**
 * Validate a stamped rigid pose without TF lookup.
 *
 * When `expected_frame` has a value, the relative frame must also match it
 * exactly. The stamp must be normalized and non-zero.
 */
[[nodiscard]] inline bool is_valid_pose_stamped(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::optional<std::string_view> expected_frame = std::nullopt)
{
  return is_canonical_relative_frame_id(pose.header.frame_id) &&
         (!expected_frame || std::string_view{pose.header.frame_id} == *expected_frame) &&
         is_normalized_nonzero_stamp(pose.header.stamp) &&
         is_valid_rigid_pose(pose.pose);
}

/**
 * Compare two rigid poses with named absolute component tolerances.
 *
 * Quaternion `q` and `-q` are treated as the same rotation. Invalid or
 * negative tolerances fail closed.
 */
[[nodiscard]] inline bool approximately_equal_rigid_pose(
  const geometry_msgs::msg::Pose & lhs,
  const geometry_msgs::msg::Pose & rhs,
  const RigidPoseComparisonTolerances tolerances = {})
{
  if (!is_valid_rigid_pose(lhs) || !is_valid_rigid_pose(rhs) ||
    !std::isfinite(tolerances.position) || tolerances.position < 0.0 ||
    !std::isfinite(tolerances.orientation_component) ||
    tolerances.orientation_component < 0.0)
  {
    return false;
  }

  const auto close = [](const double left, const double right, const double tolerance) {
      return std::abs(left - right) <= tolerance;
    };
  if (!close(lhs.position.x, rhs.position.x, tolerances.position) ||
    !close(lhs.position.y, rhs.position.y, tolerances.position) ||
    !close(lhs.position.z, rhs.position.z, tolerances.position))
  {
    return false;
  }

  const auto equal_orientation = [&](const double rhs_sign) {
      return close(
        lhs.orientation.x, rhs_sign * rhs.orientation.x,
        tolerances.orientation_component) &&
             close(
        lhs.orientation.y, rhs_sign * rhs.orientation.y,
        tolerances.orientation_component) &&
             close(
        lhs.orientation.z, rhs_sign * rhs.orientation.z,
        tolerances.orientation_component) &&
             close(
        lhs.orientation.w, rhs_sign * rhs.orientation.w,
        tolerances.orientation_component);
    };
  return equal_orientation(1.0) || equal_orientation(-1.0);
}

}  // namespace manipulation::validation

#endif  // MANIPULATION__VALIDATION__RIGID_POSE_HPP_
