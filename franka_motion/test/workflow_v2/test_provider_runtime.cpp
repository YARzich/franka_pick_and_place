// Responsibility: Verify provider runtime resource ownership without server/backend composition.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "moveit/task_constructor/task.h"

#include "workflow_v2/provider_runtime.hpp"

namespace
{

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

moveit::core::RobotModelConstPtr minimal_robot_model(const rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter<std::string>(
    "robot_description",
    "<robot name='test'>"
    "<link name='base'/>"
    "<link name='panda_grasp_frame'/>"
    "<joint name='base_to_grasp' type='fixed'>"
    "<parent link='base'/><child link='panda_grasp_frame'/>"
    "</joint>"
    "</robot>");
  node->declare_parameter<std::string>(
    "robot_description_semantic", "<robot name='test'/>");
  moveit::task_constructor::Task task("provider runtime model");
  task.loadRobotModel(node);
  return task.getRobotModel();
}

rclcpp::Node::SharedPtr node_with_provider(
  const std::string & name,
  const std::string & provider_id,
  const std::string & provider_file)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter{"workflow.observation_provider_id", provider_id},
      rclcpp::Parameter{"workflow.observation_provider_config_file", provider_file}});
  return std::make_shared<rclcpp::Node>(name, options);
}

class V2ProviderRuntimeTest : public ::testing::Test
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

TEST_F(V2ProviderRuntimeTest, OwnsDetectedSubscriptionBufferTfAndMtcModel)
{
  const auto node = node_with_provider(
    "test_v2_detected_runtime", "detected",
    "package://franka_motion/config/tasks/detected_observation.yaml");
  const auto robot_model = minimal_robot_model(node);
  const auto config = franka_motion::workflow_v2::load_v2_process_config(
    node, source_lookup());

  franka_motion::workflow_v2::ProviderRuntime runtime(node, config, robot_model);

  EXPECT_TRUE(runtime.owns_detected_transport());
  EXPECT_EQ(runtime.robot_model(), robot_model);
  EXPECT_EQ(runtime.object_catalog().instance_count(), 1U);
  EXPECT_NE(
    dynamic_cast<franka_motion::workflow_v2::DetectedObservationProvider *>(
      &runtime.observation_provider()),
    nullptr);
  EXPECT_NE(
    dynamic_cast<franka_motion::workflow_v2::MtcGraspProvider *>(
      &runtime.grasp_provider()),
    nullptr);
  runtime.request_stop();
}

TEST_F(V2ProviderRuntimeTest, ConfiguredProviderOwnsNoDetectedRosTransport)
{
  const auto node = node_with_provider(
    "test_v2_configured_runtime", "configured",
    "package://franka_motion/config/tasks/configured_observations_gazebo.yaml");
  const auto robot_model = minimal_robot_model(node);
  const auto config = franka_motion::workflow_v2::load_v2_process_config(
    node, source_lookup());

  franka_motion::workflow_v2::ProviderRuntime runtime(node, config, robot_model);

  EXPECT_FALSE(runtime.owns_detected_transport());
  EXPECT_NE(
    dynamic_cast<franka_motion::workflow_v2::ConfiguredObservationProvider *>(
      &runtime.observation_provider()),
    nullptr);
}

TEST_F(V2ProviderRuntimeTest, RejectsProviderIdAndLoadedVariantMismatch)
{
  const auto node = node_with_provider(
    "test_v2_runtime_mismatch", "detected",
    "package://franka_motion/config/tasks/detected_observation.yaml");
  const auto robot_model = minimal_robot_model(node);
  auto config = franka_motion::workflow_v2::load_v2_process_config(node, source_lookup());
  config.workflow.observation_provider_id = "configured";

  EXPECT_THROW(
    franka_motion::workflow_v2::ProviderRuntime(node, config, robot_model),
    std::invalid_argument);
}
