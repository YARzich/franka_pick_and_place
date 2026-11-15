// Responsibility: Verify strict V2 profile loading, package resources, and parameter inventory.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "workflow_v2/process_config.hpp"

namespace
{

using franka_motion::workflow_v2::LoadedV2ProcessConfig;

std::filesystem::path source_share()
{
  return FRANKA_MOTION_SOURCE_DIR;
}

franka_motion::workflow_v2::PackageShareLookup source_lookup()
{
  return [](const std::string_view package_name) {
           if (package_name != "franka_motion") {
             throw std::invalid_argument("unexpected package");
           }
           return source_share();
         };
}

std::string config_path(const std::string & relative)
{
  return (source_share() / relative).string();
}

rclcpp::Node::SharedPtr profile_node(const std::string & name)
{
  rclcpp::NodeOptions options;
  options.arguments({
      "--ros-args",
      "--params-file", config_path("config/manipulation/transfer_object_sequence_v2.yaml"),
      "--params-file", config_path("config/manipulation/robot_profile.yaml"),
      "--params-file", config_path("config/manipulation/moveit_backend.yaml"),
      "--params-file",
      config_path("config/manipulation/transfer_object_sequence_v2_deployment.yaml")});
  return std::make_shared<rclcpp::Node>(name, options);
}

void collect_keys(
  const YAML::Node & node,
  const std::string & prefix,
  std::vector<std::string> & keys)
{
  if (!node.IsMap()) {
    return;
  }
  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    const auto qualified = prefix.empty() ? key : prefix + "." + key;
    keys.push_back(qualified);
    collect_keys(entry.second, qualified, keys);
  }
}

class V2ProcessConfigTest : public ::testing::Test
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

}  // namespace

TEST_F(V2ProcessConfigTest, LoadsSeparatedShippedProfilesAndProviderFiles)
{
  const auto node = profile_node("franka_transfer_object_sequence_server");

  const LoadedV2ProcessConfig config =
    franka_motion::workflow_v2::load_v2_process_config(node, source_lookup());

  EXPECT_EQ(config.workflow.strategy_id, "pick_lift_place");
  EXPECT_EQ(config.workflow.observation_provider_id, "detected");
  EXPECT_EQ(config.workflow.grasp_provider_id, "mtc");
  EXPECT_EQ(config.workflow.max_steps, 32U);
  EXPECT_EQ(config.object_catalog.instance_count(), 1U);
  EXPECT_TRUE(std::holds_alternative<
      franka_motion::workflow_v2::DetectedObservationProviderConfig>(
      config.observation_provider));
  EXPECT_EQ(config.grasp_profiles.size(), 1U);
  EXPECT_EQ(config.robot.planning_frame, "panda_link0");
  EXPECT_EQ(config.robot.planning_group, "panda_arm");
  EXPECT_EQ(config.robot.gripper_action, "/panda_gripper/gripper_cmd");
  EXPECT_DOUBLE_EQ(config.moveit.velocity_scaling, 0.2);
  EXPECT_DOUBLE_EQ(config.moveit.acceleration_scaling, 0.2);
  EXPECT_EQ(config.deployment.sequence_action, "/transfer_object_sequence_v2");
  EXPECT_EQ(
    config.deployment.scene_renew_service,
    "/planning_scene_coordinator/renew_lease");
  EXPECT_TRUE(config.deployment.physical_attachment_service.empty());
  EXPECT_EQ(
    config.physical_actions.arm_motion,
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>{30.0}));
}

TEST_F(V2ProcessConfigTest, RejectsUnknownSelectionsBeforeRuntimeConstruction)
{
  const auto expect_unknown = [](const std::string & parameter_name, const std::string & value) {
      rclcpp::NodeOptions options;
      options.parameter_overrides({rclcpp::Parameter{parameter_name, value}});
      const auto node = std::make_shared<rclcpp::Node>(
        "test_v2_unknown_selection", options);
      EXPECT_THROW(
        franka_motion::workflow_v2::load_v2_process_config(node, source_lookup()),
        std::invalid_argument);
    };

  expect_unknown("workflow.strategy_id", "not_registered");
  expect_unknown("workflow.observation_provider_id", "camera_magic");
  expect_unknown("workflow.grasp_provider_id", "implicit_grasp");

  rclcpp::NodeOptions endpoint_options;
  endpoint_options.parameter_overrides({rclcpp::Parameter{
        "endpoints.physical_attachment_service", "relative/service"}});
  const auto endpoint_node = std::make_shared<rclcpp::Node>(
    "test_v2_invalid_physical_endpoint", endpoint_options);
  EXPECT_THROW(
    franka_motion::workflow_v2::load_v2_process_config(
      endpoint_node, source_lookup()),
    std::invalid_argument);
}

TEST_F(V2ProcessConfigTest, LoadsOptionalIsaacPhysicalAttachmentEndpoint)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter{
        "endpoints.physical_attachment_service",
        "/franka_motion/isaac/set_pick_cube_attached"}});
  const auto node = std::make_shared<rclcpp::Node>(
    "test_v2_isaac_physical_endpoint", options);

  const auto config =
    franka_motion::workflow_v2::load_v2_process_config(node, source_lookup());

  EXPECT_EQ(
    config.deployment.physical_attachment_service,
    "/franka_motion/isaac/set_pick_cube_attached");
}

TEST_F(V2ProcessConfigTest, RejectsMissingOrEscapingPackageFiles)
{
  EXPECT_THROW(
    franka_motion::workflow_v2::resolve_read_only_package_uri(
      "package://franka_motion/../outside.yaml", source_lookup()),
    std::invalid_argument);
  EXPECT_THROW(
    franka_motion::workflow_v2::resolve_read_only_package_uri(
      "file:///tmp/object_catalog.yaml", source_lookup()),
    std::invalid_argument);

  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter{
        "workflow.object_catalog_file",
        "package://franka_motion/config/tasks/does_not_exist.yaml"}});
  const auto node = std::make_shared<rclcpp::Node>("test_v2_missing_catalog", options);
  EXPECT_THROW(
    franka_motion::workflow_v2::load_v2_process_config(node, source_lookup()),
    std::invalid_argument);

  rclcpp::NodeOptions mismatch_options;
  mismatch_options.parameter_overrides({rclcpp::Parameter{
        "workflow.observation_provider_config_file",
        "package://franka_motion/config/tasks/configured_observations_gazebo.yaml"}});
  const auto mismatch_node = std::make_shared<rclcpp::Node>(
    "test_v2_mismatched_provider_file", mismatch_options);
  EXPECT_THROW(
    franka_motion::workflow_v2::load_v2_process_config(
      mismatch_node, source_lookup()),
    std::invalid_argument);
}

TEST(V2ProcessConfigInventory, AllowsOnlyTwoPhysicalTimeoutParameters)
{
  const std::vector<std::string> files{
    "config/manipulation/transfer_object_sequence_v2.yaml",
    "config/manipulation/moveit_backend.yaml",
    "config/manipulation/transfer_object_sequence_v2_deployment.yaml",
    "config/manipulation/robot_profile.yaml",
    "config/tasks/object_catalog.yaml",
    "config/tasks/configured_observations_gazebo.yaml",
    "config/tasks/configured_observations_isaac.yaml",
    "config/tasks/detected_observation.yaml",
    "config/tasks/mtc_grasp_profiles.yaml"};
  const std::set<std::string> allowed_timeouts{
    "physical_actions.arm_motion_timeout_sec",
    "physical_actions.gripper_command_timeout_sec"};
  std::set<std::string> found_timeouts;

  for (const auto & file : files) {
    std::vector<std::string> keys;
    collect_keys(YAML::LoadFile(config_path(file)), "", keys);
    for (const auto & key : keys) {
      if (key.find("timeout") != std::string::npos) {
        const auto physical = key.find("physical_actions.");
        ASSERT_NE(physical, std::string::npos) << key;
        const auto suffix = key.substr(physical);
        EXPECT_TRUE(allowed_timeouts.find(suffix) != allowed_timeouts.cend()) << key;
        found_timeouts.insert(suffix);
      }
      EXPECT_EQ(key.find("poll"), std::string::npos) << key;
      EXPECT_EQ(key.find("retry"), std::string::npos) << key;
      EXPECT_EQ(key.find("heartbeat"), std::string::npos) << key;
    }
  }
  EXPECT_EQ(found_timeouts, allowed_timeouts);

  std::ifstream task_file(config_path("config/manipulation/transfer_object_sequence_v2.yaml"));
  const std::string task_text(
    (std::istreambuf_iterator<char>(task_file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(task_text.find("use_sim_time"), std::string::npos);
}
