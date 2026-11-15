// Responsibility: Verifies joint-state freshness tracking and blocking gripper-width observation.

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "franka_motion/joint_state_monitor.hpp"

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

sensor_msgs::msg::JointState fingerState(double left, double right)
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"panda_finger_joint1", "panda_finger_joint2"};
  msg.position = {left, right};
  return msg;
}
}  // namespace

TEST_F(RclcppTest, TracksFreshJointStatesAndFingerPositions)
{
  franka_motion::FrankaJointStateMonitor monitor;
  EXPECT_FALSE(monitor.hasFreshJointState(std::chrono::seconds(1)));
  EXPECT_FALSE(monitor.gripperState().has_value());

  monitor.update(fingerState(0.02, 0.03));

  EXPECT_TRUE(monitor.hasFreshJointState(std::chrono::seconds(1)));
  const auto gripper = monitor.gripperState();
  ASSERT_TRUE(gripper.has_value());
  EXPECT_DOUBLE_EQ(gripper->left, 0.02);
  EXPECT_DOUBLE_EQ(gripper->right, 0.03);
}

TEST_F(RclcppTest, ReportsStaleJointState)
{
  franka_motion::FrankaJointStateMonitor monitor;
  monitor.update(fingerState(0.02, 0.02));

  std::this_thread::sleep_for(std::chrono::milliseconds(3));

  EXPECT_FALSE(monitor.hasFreshJointState(std::chrono::milliseconds(1)));
}

TEST_F(RclcppTest, WaitsForGripperWidth)
{
  franka_motion::FrankaJointStateMonitor monitor;
  monitor.update(fingerState(0.02, 0.02));

  EXPECT_TRUE(
    monitor.waitForGripperWidth(
      0.04,
      std::chrono::milliseconds(20),
      1.0e-9,
      false,
      "открытие",
      rclcpp::get_logger("test_joint_state_monitor")));

  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.08,
      std::chrono::milliseconds(0),
      1.0e-9,
      false,
      "открытие",
      rclcpp::get_logger("test_joint_state_monitor")));
}

TEST_F(RclcppTest, ApplicationStopInterruptsWidthWait)
{
  franka_motion::FrankaJointStateMonitor monitor;
  monitor.update(fingerState(0.02, 0.02));

  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.04,
      std::chrono::seconds(1),
      1.0e-9,
      false,
      "stopped",
      rclcpp::get_logger("test_joint_state_monitor"),
      std::chrono::milliseconds(500),
      std::nullopt,
      0.0005,
      []() {return true;}));
}

TEST_F(RclcppTest, AllowsCloseStallWhenConfigured)
{
  franka_motion::FrankaJointStateMonitor monitor;
  monitor.update(fingerState(0.015, 0.015));

  EXPECT_TRUE(
    monitor.waitForGripperWidth(
      0.0,
      std::chrono::milliseconds(0),
      1.0e-9,
      true,
      "закрытие",
      rclcpp::get_logger("test_joint_state_monitor"),
      std::chrono::milliseconds(500),
      0.08));

  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.0,
      std::chrono::milliseconds(0),
      1.0e-9,
      true,
      "закрытие без движения",
      rclcpp::get_logger("test_joint_state_monitor"),
      std::chrono::milliseconds(500),
      0.03));
}

TEST_F(RclcppTest, RejectsCloseStallWithoutFreshCompleteFingerState)
{
  franka_motion::FrankaJointStateMonitor monitor;

  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.0,
      std::chrono::milliseconds(0),
      1.0e-9,
      true,
      "закрытие",
      rclcpp::get_logger("test_joint_state_monitor")));

  auto incomplete = fingerState(0.015, 0.015);
  incomplete.name.pop_back();
  incomplete.position.pop_back();
  monitor.update(incomplete);
  EXPECT_FALSE(monitor.freshGripperState(std::chrono::seconds(1)).has_value());
  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.0,
      std::chrono::milliseconds(0),
      1.0e-9,
      true,
      "закрытие",
      rclcpp::get_logger("test_joint_state_monitor")));
}

TEST_F(RclcppTest, RejectsStallForOpeningCommand)
{
  franka_motion::FrankaJointStateMonitor monitor;
  monitor.update(fingerState(0.015, 0.015));

  EXPECT_FALSE(
    monitor.waitForGripperWidth(
      0.08,
      std::chrono::milliseconds(0),
      1.0e-9,
      true,
      "открытие",
      rclcpp::get_logger("test_joint_state_monitor")));
}
