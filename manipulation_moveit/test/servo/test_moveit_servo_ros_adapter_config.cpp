// Responsibility: Verifies fail-closed Servo adapter configuration and status classification.

#include <chrono>
#include <cstdint>
#include <string>

#include "gtest/gtest.h"
#include "moveit_msgs/msg/servo_status.hpp"

#include "manipulation_moveit/servo/moveit_servo_ros_adapter_types.hpp"

namespace manipulation::moveit
{
namespace
{

MoveItServoRosAdapterConfig valid_config()
{
  MoveItServoRosAdapterConfig config;
  config.planning_frame = "base_link";
  config.task_frame = "tool/grasp_frame";
  config.servo_tip_frame = "tool_link";
  config.joint_names = {"joint_1", "joint_2"};
  return config;
}

TEST(MoveItServoRosAdapterConfig, AcceptsCompleteRobotIndependentWiring)
{
  EXPECT_FALSE(validate_moveit_servo_ros_adapter_config(valid_config()).has_value());
}

TEST(MoveItServoRosAdapterConfig, RejectsMissingFramesJointsAndInvalidEndpoints)
{
  auto config = valid_config();
  config.planning_frame = "/base_link";
  EXPECT_NE(validate_moveit_servo_ros_adapter_config(config), std::nullopt);

  config = valid_config();
  config.joint_names.clear();
  EXPECT_NE(validate_moveit_servo_ros_adapter_config(config), std::nullopt);

  config = valid_config();
  config.joint_names.push_back("joint_1");
  EXPECT_NE(validate_moveit_servo_ros_adapter_config(config), std::nullopt);

  config = valid_config();
  config.endpoints.pose_command_topic = "bad topic";
  EXPECT_NE(validate_moveit_servo_ros_adapter_config(config), std::nullopt);
}

TEST(MoveItServoRosAdapterConfig, RejectsNonPositiveFreshnessDurations)
{
  auto config = valid_config();
  config.status_max_age = std::chrono::steady_clock::duration::zero();
  EXPECT_NE(validate_moveit_servo_ros_adapter_config(config), std::nullopt);
}

TEST(MoveItServoRosAdapterConfig, MapsEveryStableServoStatusFailClosed)
{
  using Safety = manipulation::servo::ServoSafetyStatus;
  using Status = moveit_msgs::msg::ServoStatus;

  EXPECT_EQ(classify_moveit_servo_status(Status::NO_WARNING), Safety::kNominal);
  EXPECT_EQ(
    classify_moveit_servo_status(Status::DECELERATE_FOR_APPROACHING_SINGULARITY),
    Safety::kAdvisory);
  EXPECT_EQ(
    classify_moveit_servo_status(Status::DECELERATE_FOR_LEAVING_SINGULARITY),
    Safety::kAdvisory);
  EXPECT_EQ(
    classify_moveit_servo_status(Status::DECELERATE_FOR_COLLISION),
    Safety::kAdvisory);
  EXPECT_EQ(classify_moveit_servo_status(Status::INVALID), Safety::kStopRequired);
  EXPECT_EQ(
    classify_moveit_servo_status(Status::HALT_FOR_SINGULARITY),
    Safety::kStopRequired);
  EXPECT_EQ(
    classify_moveit_servo_status(Status::HALT_FOR_COLLISION),
    Safety::kStopRequired);
  EXPECT_EQ(classify_moveit_servo_status(Status::JOINT_BOUND), Safety::kStopRequired);
  EXPECT_EQ(classify_moveit_servo_status(42), Safety::kStopRequired);
  EXPECT_FALSE(is_known_moveit_servo_status(42));
}

}  // namespace
}  // namespace manipulation::moveit
