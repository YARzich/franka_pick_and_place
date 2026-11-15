// Responsibility: Verifies robot-profile injection and last user override ordering.
// Does not own: Robot-profile values beyond the active sequence-server contract.

#include <gtest/gtest.h>

#include <string>
#include <stdexcept>
#include <vector>

#include "franka_motion/robot_profile_arguments.hpp"

namespace
{

/// Borrow stable C-string pointers from values that outlive the returned vector.
std::vector<const char *> argvFor(const std::vector<std::string> & values)
{
  std::vector<const char *> result;
  result.reserve(values.size());
  for (const auto & value : values) {
    result.push_back(value.c_str());
  }
  return result;
}

}  // namespace

TEST(RobotProfileArguments, InjectsBeforeExistingUserRosOverrides)
{
  const std::vector<std::string> source{
    "node", "application-input", "--ros-args", "-p", "planning_group:=custom_arm"};
  const auto argv = argvFor(source);

  const auto result = franka_motion::injectParameterFileBeforeRosOverrides(
    static_cast<int>(argv.size()), argv.data(), "/profile.yaml");

  EXPECT_EQ(
    result,
    (std::vector<std::string>{
    "node", "application-input", "--ros-args", "--params-file", "/profile.yaml",
    "-p", "planning_group:=custom_arm"}));
}

TEST(RobotProfileArguments, CreatesClosedRosBlockBeforePositionalArguments)
{
  const std::vector<std::string> source{"node", "application-input"};
  const auto argv = argvFor(source);

  const auto result = franka_motion::injectParameterFileBeforeRosOverrides(
    static_cast<int>(argv.size()), argv.data(), "/profile.yaml");

  EXPECT_EQ(
    result,
    (std::vector<std::string>{
    "node", "--ros-args", "--params-file", "/profile.yaml", "--",
    "application-input"}));
}

TEST(RobotProfileArguments, IgnoresRosMarkerAfterApplicationBoundary)
{
  const std::vector<std::string> source{
    "node", "application-input", "--", "--ros-args", "literal-input"};
  const auto argv = argvFor(source);

  const auto result = franka_motion::injectParameterFileBeforeRosOverrides(
    static_cast<int>(argv.size()), argv.data(), "/profile.yaml");

  EXPECT_EQ(
    result,
    (std::vector<std::string>{
    "node", "--ros-args", "--params-file", "/profile.yaml", "--",
    "application-input", "--", "--ros-args", "literal-input"}));
}

TEST(RobotProfileArguments, RejectsMalformedInputs)
{
  EXPECT_THROW(
    franka_motion::injectParameterFileBeforeRosOverrides(0, nullptr, "/profile.yaml"),
    std::invalid_argument);
  const std::vector<std::string> source{"node"};
  const auto argv = argvFor(source);
  EXPECT_THROW(
    franka_motion::injectParameterFileBeforeRosOverrides(
      static_cast<int>(argv.size()), argv.data(), ""),
    std::invalid_argument);
}

TEST(RobotProfileArguments, SupportsNamespaceAndLaterUserParameterOverride)
{
  std::vector<std::string> source{
    "test_robot_profile_arguments", "--ros-args", "-p", "robot.planning_group:=custom_arm"};
  std::vector<char *> argv;
  argv.reserve(source.size());
  for (auto & value : source) {
    argv.push_back(value.data());
  }

  franka_motion::initializeWithRobotProfile(
    static_cast<int>(argv.size()), argv.data());
  auto node = std::make_shared<rclcpp::Node>(
    "franka_transfer_object_sequence_server",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  ASSERT_TRUE(node->has_parameter("robot.planning_group"));
  EXPECT_EQ(node->get_parameter("robot.planning_group").as_string(), "custom_arm");
  ASSERT_TRUE(node->has_parameter("robot.touch_links"));
  EXPECT_EQ(node->get_parameter("robot.touch_links").as_string_array().size(), 3U);

  auto namespaced_node = std::make_shared<rclcpp::Node>(
    "franka_transfer_object_sequence_server", "/profile_review",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  ASSERT_TRUE(namespaced_node->has_parameter("robot.touch_links"));
  EXPECT_EQ(
    namespaced_node->get_parameter("robot.touch_links").as_string_array().size(), 3U);
  rclcpp::shutdown();
}
