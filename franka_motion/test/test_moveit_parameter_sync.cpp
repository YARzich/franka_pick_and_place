// Responsibility: Verifies normal MoveIt snapshot discovery and Panda limit validation.

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "franka_motion/moveit_parameter_snapshot_contract.hpp"
#include "franka_motion/moveit_parameter_sync.hpp"

#include "moveit_parameter_sync_test_support.hpp"
#include "parameter_snapshot_client.hpp"
#include "parameter_snapshot_helper_node.hpp"

namespace
{

using franka_motion::test_support::declare_or_set_parameter;
using franka_motion::test_support::minimal_sync_options;
using franka_motion::test_support::NodeSpinner;
using franka_motion::test_support::RclcppTest;
using franka_motion::test_support::set_panda_acceleration_limits;
using namespace std::chrono_literals;

TEST(MoveItParameterSnapshotContract, PreservesTransitionalMtcWireKey)
{
  EXPECT_STREQ(
    franka_motion::kMoveItParameterSnapshotManifest,
    "franka_motion_moveit.parameter_snapshot_manifest");
}

TEST_F(RclcppTest, DetectsStringParametersAndPrefixes)
{
  auto node = rclcpp::Node::make_shared("test_moveit_parameter_presence");
  node->declare_parameter<std::string>("robot_description", "robot");
  node->declare_parameter<std::string>("empty_string", "");
  node->declare_parameter<std::string>(
    "robot_description_kinematics.panda_arm.kinematics_solver", "kdl");

  EXPECT_TRUE(franka_motion::has_non_empty_string_parameter(node, "robot_description"));
  EXPECT_FALSE(franka_motion::has_non_empty_string_parameter(node, "empty_string"));
  EXPECT_FALSE(franka_motion::has_non_empty_string_parameter(node, "missing"));
  EXPECT_TRUE(franka_motion::has_parameter_prefix(node, "robot_description_kinematics"));
  EXPECT_FALSE(franka_motion::has_parameter_prefix(node, "ompl"));
}

TEST_F(RclcppTest, SnapshotHelperKeepsPrivateNameUnderGlobalLaunchRemaps)
{
  const std::vector<std::string> arguments{
    "snapshot_helper_test", "--ros-args", "--remap",
    "__node:=franka_transfer_object_sequence_server", "--remap", "__ns:=/cell"};
  std::vector<const char *> argv;
  argv.reserve(arguments.size());
  for (const auto & argument : arguments) {
    argv.push_back(argument.c_str());
  }
  auto context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  context->init(static_cast<int>(argv.size()), argv.data(), init_options);

  auto helper = franka_motion::detail::make_parameter_snapshot_helper_node(
    "franka_transfer_sequence_parameter_helper", context);

  EXPECT_STREQ(helper->get_name(), "franka_transfer_sequence_parameter_helper");
  EXPECT_STREQ(helper->get_namespace(), "/cell");
  helper.reset();
  context->shutdown("snapshot helper test completed");
}

TEST_F(RclcppTest, SnapshotClientCommunicatesInsideANonDefaultContext)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.context(context);
    auto source = rclcpp::Node::make_shared("custom_context_parameter_source", options);
    source->declare_parameter<std::string>("snapshot.value", "custom-context");
    NodeSpinner spinner(source);
    auto helper = franka_motion::detail::make_parameter_snapshot_helper_node(
      "custom_context_parameter_helper", context);
    franka_motion::detail::ParameterSnapshotClient client(
      helper, source->get_fully_qualified_name());

    ASSERT_TRUE(client.wait_for_service(1s));
    const auto values = client.get_parameters(
      {"snapshot.value"},
      franka_motion::detail::ParameterSnapshotClient::Clock::now() + 1s);
    ASSERT_TRUE(values);
    ASSERT_EQ(values->size(), 1U);
    EXPECT_EQ(values->front().as_string(), "custom-context");
  }
  context->shutdown("custom-context snapshot test completed");
}

TEST(PandaUrdfPositionLimitNames, ContainsTwoUniqueNamesForEveryJoint)
{
  auto names = franka_motion::panda_urdf_position_limit_parameter_names();
  ASSERT_EQ(names.size(), 18U);
  std::sort(names.begin(), names.end());
  EXPECT_TRUE(std::adjacent_find(names.begin(), names.end()) == names.end());
  EXPECT_TRUE(std::binary_search(
      names.begin(), names.end(),
      "robot_description_planning.joint_limits.panda_joint1.min_position"));
  EXPECT_TRUE(std::binary_search(
      names.begin(), names.end(),
      "robot_description_planning.joint_limits.panda_finger_joint2.max_position"));
}

TEST_F(RclcppTest, PublishesSortedUniqueMoveItSnapshotManifest)
{
  auto node = rclcpp::Node::make_shared("test_moveit_snapshot_manifest");
  node->declare_parameter<std::string>(
    "robot_description_kinematics.panda_arm.solver", "kdl");
  node->declare_parameter<double>(
    "robot_description_planning.joint_limits.panda_joint1.max_acceleration", 2.0);

  EXPECT_TRUE(franka_motion::publish_moveit_parameter_snapshot_manifest(
      node,
      {"robot_description_planning", "robot_description_kinematics",
        "robot_description_kinematics"}));

  const auto names = node->get_parameter(
    franka_motion::kMoveItParameterSnapshotManifest).as_string_array();
  EXPECT_EQ(
    names,
    (std::vector<std::string>{
      "robot_description_kinematics.panda_arm.solver",
      "robot_description_planning.joint_limits.panda_joint1.max_acceleration"}));
}

TEST_F(RclcppTest, RejectsEmptyMoveItSnapshotManifest)
{
  auto node = rclcpp::Node::make_shared("test_empty_moveit_snapshot_manifest");
  EXPECT_FALSE(franka_motion::publish_moveit_parameter_snapshot_manifest(
      node, {"robot_description_kinematics"}));
  EXPECT_FALSE(node->has_parameter(franka_motion::kMoveItParameterSnapshotManifest));
}

TEST_F(RclcppTest, CopiesMoveItParametersFromRemoteNode)
{
  auto source = rclcpp::Node::make_shared("test_move_group_parameter_source");
  source->declare_parameter<std::string>("robot_description", "robot");
  source->declare_parameter<std::string>("robot_description_semantic", "semantic");
  declare_or_set_parameter(source, "use_sim_time", true);
  source->declare_parameter<std::string>(
    "robot_description_kinematics.panda_arm.kinematics_solver", "kdl");
  source->declare_parameter<bool>(
    "robot_description_planning.joint_limits.panda_joint1.has_acceleration_limits",
    true);
  source->declare_parameter<double>(
    "robot_description_planning.joint_limits.panda_joint1.max_position", 2.7);
  source->declare_parameter(
    "robot_description_planning.joint_limits.panda_joint1.min_position",
    rclcpp::ParameterType::PARAMETER_DOUBLE);
  source->declare_parameter<std::vector<std::string>>(
    franka_motion::kMoveItParameterSnapshotManifest,
    {
      "robot_description_kinematics.panda_arm.kinematics_solver",
      "robot_description_planning.joint_limits.panda_joint1.has_acceleration_limits",
      "robot_description_planning.joint_limits.panda_joint1.max_position",
    });
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_moveit_parameter_destination");
  franka_motion::MoveItParameterSyncOptions options;
  options.helper_node_name = "test_moveit_parameter_helper";
  options.move_group_node_name = "test_move_group_parameter_source";
  options.service_timeout = 1s;
  options.optional_timeout = 1s;
  options.optional_names = {"use_sim_time"};
  options.required_prefixes = {"robot_description_kinematics"};
  options.optional_prefixes = {"robot_description_planning"};
  options.legacy_uninitialized_parameter_names = {
    "robot_description_planning.joint_limits.panda_joint1.max_position",
    "robot_description_planning.joint_limits.panda_joint1.min_position",
  };

  EXPECT_TRUE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_EQ(destination->get_parameter("robot_description").as_string(), "robot");
  EXPECT_EQ(
    destination->get_parameter("robot_description_semantic").as_string(), "semantic");
  EXPECT_TRUE(destination->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(
    destination->get_parameter(
      "robot_description_kinematics.panda_arm.kinematics_solver").as_string(),
    "kdl");
  EXPECT_TRUE(destination->get_parameter(
      "robot_description_planning.joint_limits.panda_joint1."
      "has_acceleration_limits").as_bool());
  EXPECT_DOUBLE_EQ(destination->get_parameter(
      "robot_description_planning.joint_limits.panda_joint1.max_position").as_double(),
    2.7);
  EXPECT_FALSE(destination->has_parameter(
      "robot_description_planning.joint_limits.panda_joint1.min_position"));
}

TEST_F(RclcppTest, ExplicitlyAbsentManifestAllowsLegacyPrefixDiscovery)
{
  auto source = rclcpp::Node::make_shared("test_legacy_parameter_source");
  source->declare_parameter<std::string>("legacy_prefix.value", "copied");
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_legacy_parameter_destination");
  auto options = minimal_sync_options(
    "test_legacy_parameter_source", "test_legacy_parameter_helper");
  options.optional_prefixes = {"legacy_prefix"};

  EXPECT_TRUE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  ASSERT_TRUE(destination->has_parameter("legacy_prefix.value"));
  EXPECT_EQ(destination->get_parameter("legacy_prefix.value").as_string(), "copied");
}

TEST_F(RclcppTest, LegacyPrefixSkipsOnlyUninitializedUrdfOwnedLimits)
{
  const std::string prefix =
    "robot_description_planning.joint_limits.panda_joint1";
  const std::string acceleration_name = prefix + ".max_acceleration";
  const std::string initialized_position_name = prefix + ".max_position";
  const std::string uninitialized_position_name = prefix + ".min_position";
  auto source = rclcpp::Node::make_shared("test_legacy_limit_source");
  source->declare_parameter<double>(acceleration_name, 2.5);
  source->declare_parameter<double>(initialized_position_name, 2.7);
  source->declare_parameter(
    uninitialized_position_name, rclcpp::ParameterType::PARAMETER_DOUBLE);
  NodeSpinner spinner(source);

  auto destination = rclcpp::Node::make_shared("test_legacy_limit_destination");
  auto options = minimal_sync_options(
    "test_legacy_limit_source", "test_legacy_limit_helper");
  options.required_prefixes = {"robot_description_planning"};
  options.legacy_uninitialized_parameter_names =
    franka_motion::panda_urdf_position_limit_parameter_names();

  EXPECT_TRUE(franka_motion::copy_moveit_parameters_from_move_group(
      destination, options));
  EXPECT_DOUBLE_EQ(destination->get_parameter(acceleration_name).as_double(), 2.5);
  EXPECT_DOUBLE_EQ(
    destination->get_parameter(initialized_position_name).as_double(), 2.7);
  EXPECT_FALSE(destination->has_parameter(uninitialized_position_name));
}

TEST_F(RclcppTest, RequiresFinitePositivePandaAccelerationLimits)
{
  auto node = rclcpp::Node::make_shared("test_panda_acceleration_limits");
  set_panda_acceleration_limits(node, 2.0);
  EXPECT_TRUE(franka_motion::has_required_panda_acceleration_limits(node));

  const std::string limit =
    "robot_description_planning.joint_limits.panda_joint4.max_acceleration";
  declare_or_set_parameter(node, limit, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(franka_motion::has_required_panda_acceleration_limits(node));

  declare_or_set_parameter(node, limit, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(franka_motion::has_required_panda_acceleration_limits(node));
}

}  // namespace
