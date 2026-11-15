// Responsibility: Converts rigid poses between geometry_msgs and Eigen representations for MTC
//   internals.

#ifndef MTC__GEOMETRY_CONVERSION_HPP_
#define MTC__GEOMETRY_CONVERSION_HPP_

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"

namespace franka_motion::detail
{

/// Convert a rigid transform to a ROS pose, normalizing its quaternion output.
inline geometry_msgs::msg::Pose eigen_to_pose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  Eigen::Quaterniond quaternion(transform.rotation());
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

/**
 * Convert a ROS pose to an Eigen transform.
 *
 * Translation is interpreted in metres. A quaternion with norm at most `1e-9`
 * leaves identity rotation; callers that require strict rigid-pose validation
 * must reject it before using this conversion.
 */
inline Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(
    pose.position.x, pose.position.y, pose.position.z);

  Eigen::Quaterniond quaternion(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  if (quaternion.norm() > 1.0e-9) {
    quaternion.normalize();
    transform.linear() = quaternion.toRotationMatrix();
  }
  return transform;
}

}  // namespace franka_motion::detail

#endif  // MTC__GEOMETRY_CONVERSION_HPP_
