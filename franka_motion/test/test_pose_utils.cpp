// Responsibility: Verifies pose/transform conversion and stamped detected-pose buffering semantics.

#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.hpp"

#include "franka_motion/pose_utils.hpp"
#include "workflow_v2/detected_pose_buffer.hpp"

namespace
{
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;
}

TEST(PoseUtils, CreatesPoseWithCorrectPosition)
{
  const auto pose = franka_motion::makePose(0.1, -0.2, 0.3, 0.0, 0.0, 0.0);
  EXPECT_NEAR(pose.position.x, 0.1, 1e-9);
  EXPECT_NEAR(pose.position.y, -0.2, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.3, 1e-9);
  EXPECT_NEAR(pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.w, 1.0, 1e-9);
}

TEST(PoseUtils, CreatesPoseWithYawRotation)
{
  const auto pose = franka_motion::makePose(0.0, 0.0, 0.0, 0.0, 0.0, kPi / 2.0);
  const double expected = std::sin(kPi / 4.0);
  EXPECT_NEAR(pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.z, expected, 1e-6);
  EXPECT_NEAR(pose.orientation.w, expected, 1e-6);
}

TEST(PoseUtils, ConvertsPoseToTransformWithFallbackIdentityRotation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -0.2;
  pose.position.z = 0.3;

  const auto transform = franka_motion::poseToTransform(pose);

  EXPECT_NEAR(transform.getOrigin().x(), 0.1, 1e-9);
  EXPECT_NEAR(transform.getOrigin().y(), -0.2, 1e-9);
  EXPECT_NEAR(transform.getOrigin().z(), 0.3, 1e-9);
  EXPECT_NEAR(transform.getRotation().x(), 0.0, 1e-9);
  EXPECT_NEAR(transform.getRotation().y(), 0.0, 1e-9);
  EXPECT_NEAR(transform.getRotation().z(), 0.0, 1e-9);
  EXPECT_NEAR(transform.getRotation().w(), 1.0, 1e-9);
}

TEST(PoseUtils, ConvertsTransformToPose)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(0.4, 0.5, 0.6));
  tf2::Quaternion rotation;
  rotation.setRPY(kPi, 0.0, 0.0);
  transform.setRotation(rotation);

  const auto pose = franka_motion::transformToPose(transform);

  EXPECT_NEAR(pose.position.x, 0.4, 1e-9);
  EXPECT_NEAR(pose.position.y, 0.5, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.6, 1e-9);
  EXPECT_NEAR(std::abs(pose.orientation.x), 1.0, 1e-9);
  EXPECT_NEAR(pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.orientation.z, 0.0, 1e-9);
}

TEST(PoseUtils, BuildsLocalTransformOrUsesFallbackForInvalidParameters)
{
  tf2::Transform fallback;
  fallback.setIdentity();
  fallback.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));

  const auto valid = franka_motion::makeLocalTransform(
    {0.1, 0.2, 0.3},
    {0.0, 0.0, 0.0},
    fallback,
    rclcpp::get_logger("test_pose_utils"),
    "valid");
  EXPECT_NEAR(valid.getOrigin().x(), 0.1, 1e-9);

  const auto invalid = franka_motion::makeLocalTransform(
    {std::numeric_limits<double>::quiet_NaN(), 0.2, 0.3},
    {0.0, 0.0, 0.0},
    fallback,
    rclcpp::get_logger("test_pose_utils"),
    "invalid");
  EXPECT_NEAR(invalid.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(invalid.getOrigin().y(), 2.0, 1e-9);
  EXPECT_NEAR(invalid.getOrigin().z(), 3.0, 1e-9);
}

TEST(PoseUtils, CreatesStampedAndTranslatedPoses)
{
  const auto pose = franka_motion::makePose(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
  const rclcpp::Time stamp(123, 456);

  const auto stamped = franka_motion::makePoseStamped(pose, "panda_link0", stamp);
  EXPECT_EQ(stamped.header.frame_id, "panda_link0");
  EXPECT_EQ(stamped.header.stamp, stamp);
  EXPECT_NEAR(stamped.pose.position.x, 0.1, 1e-9);

  const auto translated = franka_motion::translatedPose(stamped, 1.0, -2.0, 3.0, stamp);
  EXPECT_EQ(translated.header.frame_id, "panda_link0");
  EXPECT_NEAR(translated.pose.position.x, 1.1, 1e-9);
  EXPECT_NEAR(translated.pose.position.y, -1.8, 1e-9);
  EXPECT_NEAR(translated.pose.position.z, 3.3, 1e-9);
}

TEST(PoseUtils, TransformsAtTheCorrelatedMeasurementStamp)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "panda_link0";
  transform.child_frame_id = "camera";
  transform.header.stamp.sec = 10;
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test_pose_utils"));

  transform.header.stamp.sec = 11;
  transform.transform.translation.x = 2.0;
  ASSERT_TRUE(buffer.setTransform(transform, "test_pose_utils"));

  geometry_msgs::msg::PoseStamped detected;
  detected.header.frame_id = "camera";
  detected.header.stamp.sec = 10;
  detected.pose.orientation.w = 1.0;
  const auto transformed = franka_motion::transformPoseToFrame(
    detected, "panda_link0", "", buffer,
    rclcpp::get_logger("test_pose_utils"), 0.01);

  ASSERT_TRUE(transformed.has_value());
  EXPECT_EQ(transformed->header.frame_id, "panda_link0");
  EXPECT_EQ(transformed->header.stamp, detected.header.stamp);
  EXPECT_NEAR(transformed->pose.position.x, 1.0, 1e-9);

  detected.header.stamp.sec = 9;
  EXPECT_FALSE(
    franka_motion::transformPoseToFrame(
      detected, "panda_link0", "", buffer,
      rclcpp::get_logger("test_pose_utils"), 0.001).has_value());
}

TEST(PoseUtils, ValidatesFramesBeforeSameFrameShortcutAndEmptyFallback)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "robot//camera";
  pose.header.stamp.sec = 1;
  pose.pose.orientation.w = 1.0;
  const auto logger = rclcpp::get_logger("test_pose_utils");

  EXPECT_FALSE(franka_motion::transformPoseToFrame(
      pose, pose.header.frame_id, "", buffer, logger, 0.0));

  pose.header.frame_id.clear();
  EXPECT_FALSE(franka_motion::transformPoseToFrame(
      pose, "world", "world/", buffer, logger, 0.0));
  const auto fallback_same_frame = franka_motion::transformPoseToFrame(
    pose, "world", "world", buffer, logger, 0.0);
  ASSERT_TRUE(fallback_same_frame);
  EXPECT_EQ(fallback_same_frame->header.frame_id, "world");

  pose.header.frame_id = "robot/camera";
  const auto same_frame = franka_motion::transformPoseToFrame(
    pose, pose.header.frame_id, "unused//fallback", buffer, logger, 0.0);
  ASSERT_TRUE(same_frame);
  EXPECT_EQ(same_frame->header, pose.header);
}

TEST(PoseUtils, CorrelatesPoseByNonzeroStampAndExactFrame)
{
  geometry_msgs::msg::PoseStamped cube;
  cube.header.frame_id = "world";
  cube.header.stamp.sec = 12;
  cube.header.stamp.nanosec = 34;
  auto grasp = cube;

  EXPECT_TRUE(franka_motion::hasCorrelationHeader(cube));
  EXPECT_TRUE(franka_motion::hasSameCorrelationHeader(cube, grasp));

  grasp.header.frame_id = "panda_link0";
  EXPECT_FALSE(franka_motion::hasSameCorrelationHeader(cube, grasp));
  grasp = cube;
  ++grasp.header.stamp.nanosec;
  EXPECT_FALSE(franka_motion::hasSameCorrelationHeader(cube, grasp));
  grasp = cube;
  grasp.header.stamp.sec = 0;
  grasp.header.stamp.nanosec = 0;
  EXPECT_FALSE(franka_motion::hasSameCorrelationHeader(cube, grasp));

  for (const std::string & invalid_frame :
    {"world/", "robot//camera", "world\ncamera"})
  {
    cube.header.frame_id = invalid_frame;
    EXPECT_FALSE(franka_motion::hasCorrelationHeader(cube));
  }
}

TEST(PoseUtils, ValidatesFinitePoseAndQuaternionNorm)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  EXPECT_TRUE(franka_motion::isValidPose(pose));

  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(franka_motion::isValidPose(pose));

  pose.position.x = 0.0;
  pose.orientation.w = 0.0;
  EXPECT_FALSE(franka_motion::isValidPose(pose));

  pose.orientation.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(franka_motion::isValidPose(pose));

  pose.orientation.z = std::numeric_limits<double>::max();
  pose.orientation.w = std::numeric_limits<double>::max();
  EXPECT_TRUE(franka_motion::isValidPose(pose));

  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0e-7;
  EXPECT_FALSE(franka_motion::isValidPose(pose));
}

TEST(PoseUtils, NormalizesValidCorrelatedPoseAndFallbackFrame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp.sec = 12;
  pose.header.stamp.nanosec = 34;
  pose.pose.position.x = 0.1;
  pose.pose.orientation.w = 2.0;

  const auto normalized = franka_motion::normalizeCorrelatedPose(pose, "world");

  ASSERT_TRUE(normalized.has_value());
  EXPECT_EQ(normalized->header.frame_id, "world");
  EXPECT_EQ(normalized->header.stamp, pose.header.stamp);
  EXPECT_NEAR(normalized->pose.orientation.w, 1.0, 1e-12);
}

TEST(PoseUtils, RejectsMalformedCorrelationHeaderAndPose)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp.sec = 1;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(franka_motion::normalizeCorrelatedPose(pose).has_value());

  pose.header.stamp.sec = -1;
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose).has_value());
  pose.header.stamp.sec = 1;
  pose.header.stamp.nanosec = 1000000000U;
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose).has_value());
  pose.header.stamp.nanosec = 0;
  pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose).has_value());
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose).has_value());

  pose.pose.orientation.w = 1.0;
  pose.header.frame_id.clear();
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose, "world/").has_value());
  pose.header.frame_id = "robot//camera";
  EXPECT_FALSE(franka_motion::normalizeCorrelatedPose(pose, "world").has_value());
}

TEST(PoseUtils, ComputesRotationDeltaWithoutQuaternionOverflow)
{
  geometry_msgs::msg::PoseStamped previous;
  previous.header.frame_id = "world";
  previous.header.stamp.sec = 1;
  const double huge = std::numeric_limits<double>::max();
  previous.pose.orientation.x = huge;
  previous.pose.orientation.y = huge;
  previous.pose.orientation.z = huge;
  previous.pose.orientation.w = huge;
  auto current = previous;
  current.header.stamp.sec = 2;

  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, true, false, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kRepublish);
}

TEST(DetectedPoseBuffer, ReturnsOnlyCubePoseReceivedAfterRequest)
{
  franka_motion::DetectedPoseBuffer buffer;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp.sec = 1;

  const auto request_time = franka_motion::DetectedPoseBuffer::SteadyClock::now();
  buffer.updateCubePose(pose, request_time - 1s);
  EXPECT_FALSE(buffer.waitForFreshCubePose(1ms, request_time, []() {return true;}).has_value());

  pose.header.stamp.sec = 2;
  buffer.updateCubePose(pose, request_time);
  const auto fresh_pose = buffer.waitForFreshCubePose(
    1ms, request_time, []() {return true;});
  ASSERT_TRUE(fresh_pose.has_value());
  EXPECT_EQ(fresh_pose->header.stamp.sec, 2);
}

TEST(DetectedPoseBuffer, StopsWaitingWhenCallerContextIsNotRunning)
{
  franka_motion::DetectedPoseBuffer buffer;
  const auto start = franka_motion::DetectedPoseBuffer::SteadyClock::now();
  EXPECT_FALSE(
    buffer.waitForFreshCubePose(1s, start, []() {return false;}).has_value());
}

TEST(PoseUtils, RepublishesSuccessfulGraspForNewCorrelationHeader)
{
  geometry_msgs::msg::PoseStamped previous;
  previous.header.frame_id = "world";
  previous.header.stamp.sec = 10;
  previous.pose.orientation.w = 1.0;

  auto current = previous;
  current.header.stamp.sec = 11;
  current.pose.position.x = 0.001;

  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, true, false, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kRepublish);

  current.header = previous.header;
  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, true, false, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kIgnore);
}

TEST(PoseUtils, ReplansForMeaningfulPoseChangeOrFailedRetry)
{
  geometry_msgs::msg::PoseStamped previous;
  previous.header.frame_id = "world";
  previous.header.stamp.sec = 10;
  previous.pose.orientation.w = 1.0;

  auto current = previous;
  current.header.stamp.sec = 11;
  current.pose.position.x = 0.01;
  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, true, false, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kReplan);

  current = previous;
  current.header.frame_id = "panda_link0";
  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, true, false, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kReplan);

  current = previous;
  current.header.stamp.sec = 11;
  EXPECT_EQ(
    franka_motion::chooseCorrelatedPoseAction(current, previous, false, true, 0.01, 0.1),
    franka_motion::CorrelatedPoseAction::kReplan);
}
