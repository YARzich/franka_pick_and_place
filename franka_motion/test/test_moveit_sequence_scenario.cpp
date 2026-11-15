// Responsibility: Verifies MoveIt diagnostic configuration, ordering, and failure semantics.
// Does not own: MoveGroup, controllers, executor threads, or simulator integration.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "moveit_sequence_scenario.hpp"

namespace
{
using namespace std::chrono_literals;

class MoveItSequenceScenarioRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

/// Create fake operations that record every requested diagnostic stage.
franka_motion::MoveItSequenceOperations recordingOperations(
  std::vector<std::string> & events,
  const bool first_pose_succeeds = true,
  const bool second_pose_succeeds = true,
  const bool return_succeeds = true)
{
  franka_motion::MoveItSequenceOperations operations;
  operations.capture_initial_joints = [&events]() {
      events.push_back("capture");
      return std::optional<std::vector<double>>{{0.1, -0.2, 0.3}};
    };
  operations.move_to_pose =
    [&events, first_pose_succeeds, second_pose_succeeds](
    const auto &, const std::string & tag)
    {
      events.push_back(tag);
      return tag == "target_pose_1" ? first_pose_succeeds : second_pose_succeeds;
    };
  operations.move_to_joints = [&events, return_succeeds](
    const std::vector<double> & joints, const std::string & tag)
    {
      events.push_back(tag);
      EXPECT_EQ(joints, (std::vector<double>{0.1, -0.2, 0.3}));
      return return_succeeds;
    };
  return operations;
}

}  // namespace

TEST_F(MoveItSequenceScenarioRosTest, ReadsDocumentedDefaultsAndFixedTargets)
{
  auto node = rclcpp::Node::make_shared("test_moveit_sequence_defaults");
  const auto config = franka_motion::readMoveItSequenceConfig(node);

  EXPECT_EQ(config.planning_group, "panda_arm");
  EXPECT_EQ(config.reference_frame, "panda_link0");
  EXPECT_DOUBLE_EQ(config.planning_time_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.velocity_scaling, 0.2);
  EXPECT_DOUBLE_EQ(config.acceleration_scaling, 0.2);
  EXPECT_EQ(config.current_state_timeout, 10s);
  EXPECT_EQ(config.joint_state_max_age, 500ms);

  const auto targets = franka_motion::defaultMoveItSequenceTargets();
  EXPECT_DOUBLE_EQ(targets[0].position.x, 0.45);
  EXPECT_DOUBLE_EQ(targets[0].position.y, 0.0);
  EXPECT_DOUBLE_EQ(targets[0].position.z, 0.35);
  EXPECT_DOUBLE_EQ(targets[1].position.x, 0.35);
  EXPECT_DOUBLE_EQ(targets[1].position.y, 0.2);
  EXPECT_DOUBLE_EQ(targets[1].position.z, 0.4);
  for (const auto & target : targets) {
    const double norm =
      target.orientation.x * target.orientation.x +
      target.orientation.y * target.orientation.y +
      target.orientation.z * target.orientation.z +
      target.orientation.w * target.orientation.w;
    EXPECT_NEAR(norm, 1.0, 1e-12);
  }
}

TEST_F(MoveItSequenceScenarioRosTest, ExecutesBothPosesThenReturnsToCapturedJoints)
{
  std::vector<std::string> events;
  const auto operations = recordingOperations(events);

  EXPECT_TRUE(franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")));
  EXPECT_EQ(
    events,
    (std::vector<std::string>{
    "capture", "target_pose_1", "target_pose_2", "initial_joint_state"}));
}

TEST_F(MoveItSequenceScenarioRosTest, StopsAfterFirstFailedPose)
{
  std::vector<std::string> events;
  const auto operations = recordingOperations(events, false);

  EXPECT_FALSE(franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")));
  EXPECT_EQ(events, (std::vector<std::string>{"capture", "target_pose_1"}));
}

TEST_F(MoveItSequenceScenarioRosTest, StopsAfterSecondFailedPoseWithoutRecovery)
{
  std::vector<std::string> events;
  const auto operations = recordingOperations(events, true, false);

  EXPECT_FALSE(franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")));
  EXPECT_EQ(
    events,
    (std::vector<std::string>{"capture", "target_pose_1", "target_pose_2"}));
}

TEST_F(MoveItSequenceScenarioRosTest, ReportsReturnFailureAsSequenceFailure)
{
  std::vector<std::string> events;
  const auto operations = recordingOperations(events, true, true, false);

  EXPECT_FALSE(franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")));
  EXPECT_EQ(events.back(), "initial_joint_state");
}

TEST_F(MoveItSequenceScenarioRosTest, RejectsMissingOrMalformedInitialState)
{
  franka_motion::MoveItSequenceOperations operations;
  operations.capture_initial_joints = []() {
      return std::optional<std::vector<double>>{
      {std::numeric_limits<double>::quiet_NaN()}};
    };
  operations.move_to_pose = [](const auto &, const auto &) {return true;};
  operations.move_to_joints = [](const auto &, const auto &) {return true;};

  EXPECT_FALSE(franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")));
  operations.move_to_joints = {};
  EXPECT_THROW(
    franka_motion::runMoveItSequence(
      operations, franka_motion::defaultMoveItSequenceTargets(),
      rclcpp::get_logger("test_moveit_sequence")),
    std::invalid_argument);
}

TEST_F(MoveItSequenceScenarioRosTest, RejectsInvalidExplicitParameters)
{
  auto node = std::make_shared<rclcpp::Node>(
    "test_moveit_sequence_invalid_group",
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("planning_group", ""),
  }));

  EXPECT_THROW(
    franka_motion::readMoveItSequenceConfig(node),
    std::invalid_argument);

  node = std::make_shared<rclcpp::Node>(
    "test_moveit_sequence_invalid_time",
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("planning_time", -1.0),
  }));
  EXPECT_THROW(
    franka_motion::readMoveItSequenceConfig(node),
    std::invalid_argument);

  std::size_t invalid_frame_index = 0U;
  for (const std::string & invalid_frame :
    {"/absolute_frame", "panda_link0/", "robot//base", "world\nframe"})
  {
    node = std::make_shared<rclcpp::Node>(
      "test_moveit_sequence_invalid_frame_" +
      std::to_string(invalid_frame_index++),
      rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("reference_frame", invalid_frame),
    }));
    EXPECT_THROW(
      franka_motion::readMoveItSequenceConfig(node),
      std::invalid_argument);
  }
}
