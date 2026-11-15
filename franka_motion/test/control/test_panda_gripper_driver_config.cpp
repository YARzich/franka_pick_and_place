// Responsibility: Verifies Panda gripper driver parameter defaults and safety constraints.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "panda_gripper_driver_config.hpp"

namespace
{
class RclcppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

std::shared_ptr<rclcpp::Node> makeNode(
  const std::string & name,
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  return std::make_shared<rclcpp::Node>(name, options);
}
}  // namespace

TEST_F(RclcppTest, PreservesPublicParameterDefaultsAndUnits)
{
  auto node = makeNode("default_gripper_config");
  const auto config = franka_motion::declareAndReadPandaGripperDriverConfig(*node);

  EXPECT_EQ(config.command_topic, "/panda_hand_controller/commands");
  EXPECT_EQ(config.action_name, "/panda_gripper/gripper_cmd");
  EXPECT_EQ(config.joint_states_topic, "/joint_states");
  EXPECT_DOUBLE_EQ(config.goal_policy.goal_tolerance, 0.004);
  EXPECT_DOUBLE_EQ(config.goal_policy.movement_threshold, 0.0005);
  EXPECT_EQ(config.goal_policy.state_timeout, std::chrono::milliseconds(500));
  EXPECT_EQ(config.goal_policy.stall_timeout, std::chrono::milliseconds(500));
  EXPECT_EQ(config.goal_policy.goal_timeout, std::chrono::seconds(3));
  EXPECT_TRUE(config.goal_policy.allow_stalling);
  EXPECT_EQ(config.monitor_period, std::chrono::milliseconds(50));
  EXPECT_EQ(config.command_joint_count, 2);
  EXPECT_DOUBLE_EQ(config.command_joint_min_position, 0.0);
  EXPECT_DOUBLE_EQ(config.command_joint_max_position, 0.04);
  EXPECT_DOUBLE_EQ(config.goal_policy.finger_symmetry_tolerance, 0.01);
  EXPECT_DOUBLE_EQ(config.goal_policy.maximum_reverse_motion, 0.002);
}

TEST_F(RclcppTest, AppliesOverridesAndExistingMinimumTimeoutPolicy)
{
  auto node = makeNode(
    "overridden_gripper_config",
  {
    rclcpp::Parameter("command_joint_count", 1),
    rclcpp::Parameter("command_joint_min_position", 0.001),
    rclcpp::Parameter("command_joint_max_position", 0.039),
    rclcpp::Parameter("state_timeout_sec", -1.0),
    rclcpp::Parameter("allow_stalling", false),
    });
  const auto config = franka_motion::declareAndReadPandaGripperDriverConfig(*node);

  EXPECT_EQ(config.command_joint_count, 1);
  EXPECT_DOUBLE_EQ(config.command_joint_min_position, 0.001);
  EXPECT_DOUBLE_EQ(config.command_joint_max_position, 0.039);
  EXPECT_EQ(config.goal_policy.state_timeout, std::chrono::milliseconds(50));
  EXPECT_FALSE(config.goal_policy.allow_stalling);
}

TEST_F(RclcppTest, RejectsInvalidCommandShapeAndJointLimits)
{
  auto invalid_count = makeNode(
    "invalid_gripper_command_count",
    {rclcpp::Parameter("command_joint_count", 3)});
  EXPECT_THROW(
    franka_motion::declareAndReadPandaGripperDriverConfig(*invalid_count),
    std::invalid_argument);

  auto invalid_limits = makeNode(
    "invalid_gripper_joint_limits",
  {
    rclcpp::Parameter("command_joint_min_position", 0.03),
    rclcpp::Parameter("command_joint_max_position", 0.02),
    });
  EXPECT_THROW(
    franka_motion::declareAndReadPandaGripperDriverConfig(*invalid_limits),
    std::invalid_argument);
}

TEST_F(RclcppTest, RejectsNonFinitePolicyThresholds)
{
  auto node = makeNode(
    "invalid_gripper_policy",
    {rclcpp::Parameter(
        "finger_symmetry_tolerance",
        std::numeric_limits<double>::quiet_NaN())});
  EXPECT_THROW(
    franka_motion::declareAndReadPandaGripperDriverConfig(*node),
    std::invalid_argument);
}

TEST_F(RclcppTest, RejectsEmptyOrInvalidRosInterfaceNames)
{
  auto empty_action = makeNode(
    "empty_gripper_action_name",
    {rclcpp::Parameter("action_name", std::string{})});
  EXPECT_THROW(
    franka_motion::declareAndReadPandaGripperDriverConfig(*empty_action),
    std::invalid_argument);

  auto invalid_topic = makeNode(
    "invalid_gripper_command_topic",
    {rclcpp::Parameter("command_topic", "bad topic")});
  EXPECT_THROW(
    franka_motion::declareAndReadPandaGripperDriverConfig(*invalid_topic),
    std::invalid_argument);
}
