// Responsibility: Verifies lossless pose/Eigen conversion and malformed quaternion fallback.

#include <Eigen/Geometry>

#include <cmath>

#include "gtest/gtest.h"

#include "geometry_conversion.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

TEST(EigenPoseUtils, RoundTripsRigidTransform)
{
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  expected.linear() =
    Eigen::AngleAxisd(kPi / 3.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto pose = franka_motion::detail::eigen_to_pose(expected);
  const auto actual = franka_motion::detail::pose_to_eigen(pose);

  EXPECT_TRUE(actual.isApprox(expected, 1e-12));
}

TEST(EigenPoseUtils, UsesIdentityForZeroQuaternion)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;

  const auto transform = franka_motion::detail::pose_to_eigen(pose);

  EXPECT_TRUE(transform.translation().isApprox(Eigen::Vector3d(0.4, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(transform.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}
