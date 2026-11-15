// Responsibility: Verify every validated V2 profile value reaches its concrete adapter config.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"

#include "workflow_v2/composition.hpp"
#include "workflow_v2/process_config.hpp"

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

std::string config_path(const std::string & relative)
{
  return (source_share() / relative).string();
}

rclcpp::Node::SharedPtr config_node(const std::string & planning_pipeline = "")
{
  rclcpp::NodeOptions options;
  options.arguments({
      "--ros-args",
      "--params-file", config_path("config/manipulation/transfer_object_sequence_v2.yaml"),
      "--params-file", config_path("config/manipulation/robot_profile.yaml"),
      "--params-file", config_path("config/manipulation/moveit_backend.yaml"),
      "--params-file",
      config_path("config/manipulation/transfer_object_sequence_v2_deployment.yaml")});
  if (!planning_pipeline.empty()) {
    options.parameter_overrides({
        rclcpp::Parameter{"moveit.planning_pipeline", planning_pipeline}});
  }
  return std::make_shared<rclcpp::Node>(
    "franka_transfer_object_sequence_server", options);
}

class V2CompositionContractTest : public ::testing::Test
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

TEST_F(V2CompositionContractTest, MapsAllProcessOwnershipBoundariesWithoutRosEndpoints)
{
  auto config = franka_motion::workflow_v2::load_v2_process_config(
    config_node(), source_lookup());
  const auto wait_logger =
    std::make_shared<manipulation::workflow::WaitLifecycleLogger>(
    [](manipulation::workflow::WaitLifecycleRecord) {});

  const auto bindings =
    franka_motion::workflow_v2::detail::make_composition_bindings(
    config, wait_logger);

  EXPECT_EQ(bindings.scene.attachment_link, config.robot.attachment_link);
  EXPECT_EQ(bindings.scene.touch_links, config.robot.touch_links);
  EXPECT_EQ(bindings.scene.acquire_service, config.deployment.scene_acquire_service);
  EXPECT_EQ(bindings.scene.renew_service, config.deployment.scene_renew_service);
  EXPECT_EQ(bindings.scene.release_service, config.deployment.scene_release_service);
  EXPECT_EQ(bindings.scene.validate_service, config.deployment.scene_validate_service);
  EXPECT_EQ(bindings.scene.state_service, config.deployment.scene_state_service);
  EXPECT_EQ(bindings.scene.apply_diff_service, config.deployment.scene_apply_service);
  EXPECT_EQ(
    bindings.scene.get_planning_scene_service,
    config.deployment.planning_scene_read_service);
  EXPECT_EQ(
    bindings.physical_scene.service_name,
    config.deployment.physical_attachment_service);
  EXPECT_EQ(bindings.physical_scene.object_id, "cube");

  EXPECT_EQ(bindings.planning.move_group_action, config.moveit.move_group_action);
  EXPECT_EQ(bindings.planning.planning_group, config.robot.planning_group);
  EXPECT_EQ(bindings.planning.end_effector_link, config.robot.end_effector_link);
  EXPECT_EQ(bindings.planning.planning_pipeline, config.moveit.planning_pipeline);
  EXPECT_EQ(bindings.planning.planner_id, config.moveit.planner_id);
  EXPECT_DOUBLE_EQ(bindings.planning.velocity_scaling, config.moveit.velocity_scaling);
  EXPECT_DOUBLE_EQ(
    bindings.planning.acceleration_scaling, config.moveit.acceleration_scaling);
  EXPECT_DOUBLE_EQ(bindings.planning.position_tolerance, config.moveit.position_tolerance);
  EXPECT_DOUBLE_EQ(
    bindings.planning.orientation_tolerance, config.moveit.orientation_tolerance);
  EXPECT_DOUBLE_EQ(bindings.planning.joint_tolerance, config.moveit.joint_tolerance);

  EXPECT_EQ(bindings.strategy.robot.attachment_link, config.robot.attachment_link);
  EXPECT_EQ(bindings.strategy.robot.touch_links, config.robot.touch_links);
  EXPECT_DOUBLE_EQ(bindings.strategy.robot.open_width_m, config.robot.open_width_m);
  EXPECT_EQ(bindings.strategy.robot.allow_grasp_stall, config.robot.allow_grasp_stall);
  EXPECT_EQ(bindings.strategy.robot.home_joint_names, config.robot.home_joint_names);
  EXPECT_EQ(bindings.strategy.robot.home_joint_positions, config.robot.home_joint_positions);
  EXPECT_DOUBLE_EQ(
    bindings.strategy.task.lift_clearance_m,
    config.pick_lift_place.lift_clearance_m);
  EXPECT_DOUBLE_EQ(
    bindings.strategy.task.preplace_clearance_m,
    config.pick_lift_place.preplace_clearance_m);
  EXPECT_DOUBLE_EQ(
    bindings.strategy.task.retreat_clearance_m,
    config.pick_lift_place.retreat_clearance_m);
  EXPECT_EQ(
    bindings.strategy.task.release_settle,
    config.pick_lift_place.release_settle);
  EXPECT_EQ(bindings.strategy.arm_motion_timeout, config.physical_actions.arm_motion);
  EXPECT_EQ(
    bindings.strategy.gripper_command_timeout,
    config.physical_actions.gripper_command);

  EXPECT_EQ(
    bindings.trajectory_execution.action_name,
    config.moveit.execute_trajectory_action);
  EXPECT_EQ(bindings.gripper.action_name, config.robot.gripper_action);
  EXPECT_EQ(bindings.health.topic, config.deployment.health_topic);
  EXPECT_EQ(bindings.workflow.validation.max_step_count, config.workflow.max_steps);
  EXPECT_EQ(bindings.workflow.validation.planning_frame, config.robot.planning_frame);
  EXPECT_EQ(bindings.workflow.wait_logger, wait_logger);
  EXPECT_EQ(bindings.action_server.action_name, config.deployment.sequence_action);
  EXPECT_EQ(
    bindings.action_server.pause_service_name,
    config.deployment.sequence_pause_service);

  const auto & policies = bindings.workflow.validation.supported_recovery_policies;
  EXPECT_EQ(policies.size(), 4U);
  EXPECT_EQ(policies.front(), manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_NONE);
  EXPECT_EQ(
    policies.back(),
    manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_RETREAT_AND_HOME);
}

TEST_F(V2CompositionContractTest, RequiresTheProductionWaitLogger)
{
  const auto config = franka_motion::workflow_v2::load_v2_process_config(
    config_node(), source_lookup());

  EXPECT_THROW(
    franka_motion::workflow_v2::detail::make_composition_bindings(config, nullptr),
    std::invalid_argument);
}

TEST_F(V2CompositionContractTest, MapsEveryWaitExitOutcomeToStableLogFields)
{
  using manipulation::workflow::WaitLifecycleOutcome;
  using franka_motion::workflow_v2::detail::wait_lifecycle_outcome_name;

  EXPECT_STREQ(wait_lifecycle_outcome_name(std::nullopt), "none");
  EXPECT_STREQ(wait_lifecycle_outcome_name(WaitLifecycleOutcome::kReady), "ready");
  EXPECT_STREQ(
    wait_lifecycle_outcome_name(WaitLifecycleOutcome::kCanceled), "canceled");
  EXPECT_STREQ(
    wait_lifecycle_outcome_name(WaitLifecycleOutcome::kShutdown), "shutdown");
  EXPECT_STREQ(wait_lifecycle_outcome_name(WaitLifecycleOutcome::kError), "error");
}

TEST_F(V2CompositionContractTest, CuRoboProfileChangesOnlyTheCommonPlanningPipeline)
{
  const auto make_bindings = [](const std::string & pipeline) {
      const auto config = franka_motion::workflow_v2::load_v2_process_config(
        config_node(pipeline), source_lookup());
      const auto wait_logger =
        std::make_shared<manipulation::workflow::WaitLifecycleLogger>(
        [](manipulation::workflow::WaitLifecycleRecord) {});
      return franka_motion::workflow_v2::detail::make_composition_bindings(
        config, wait_logger);
    };

  const auto ompl = make_bindings("ompl");
  const auto curobo = make_bindings("curobo");

  EXPECT_EQ(ompl.planning.planning_pipeline, "ompl");
  EXPECT_EQ(curobo.planning.planning_pipeline, "curobo");
  EXPECT_EQ(curobo.planning.move_group_action, ompl.planning.move_group_action);
  EXPECT_EQ(curobo.planning.planning_group, ompl.planning.planning_group);
  EXPECT_EQ(curobo.planning.end_effector_link, ompl.planning.end_effector_link);
  EXPECT_EQ(curobo.trajectory_execution.action_name, ompl.trajectory_execution.action_name);
  EXPECT_EQ(curobo.gripper.action_name, ompl.gripper.action_name);
  EXPECT_EQ(curobo.scene.apply_diff_service, ompl.scene.apply_diff_service);
  EXPECT_EQ(curobo.action_server.action_name, ompl.action_server.action_name);
  EXPECT_EQ(
    curobo.action_server.pause_service_name,
    ompl.action_server.pause_service_name);
  EXPECT_EQ(curobo.workflow.validation.max_step_count, ompl.workflow.validation.max_step_count);
}
