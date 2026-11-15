// Responsibility: Verifies required ServoToPose loading, validation, and identity coverage.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_moveit/config/moveit_servo_process_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::moveit
{
namespace
{

using Parameters = std::vector<rclcpp::Parameter>;
using namespace std::chrono_literals;

class ConfigNode final
{
public:
  ConfigNode(std::string name, const Parameters & parameters)
  : context_(std::make_shared<rclcpp::Context>())
  {
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    options.parameter_overrides(parameters);
    node_ = std::make_unique<rclcpp::Node>(std::move(name), options);
  }

  ~ConfigNode()
  {
    node_.reset();
    context_->shutdown("MoveIt Servo process config test complete");
  }

  [[nodiscard]] rclcpp::Node & node() const
  {
    return *node_;
  }

private:
  std::shared_ptr<rclcpp::Context> context_;
  std::unique_ptr<rclcpp::Node> node_;
};

void set_override(Parameters & parameters, rclcpp::Parameter replacement)
{
  const auto found = std::find_if(
    parameters.begin(), parameters.end(), [&replacement](const auto & parameter) {
      return parameter.get_name() == replacement.get_name();
    });
  if (found == parameters.end()) {
    parameters.push_back(std::move(replacement));
  } else {
    *found = std::move(replacement);
  }
}

Parameters complete_profile()
{
  return {
    rclcpp::Parameter{"servo_to_pose.preset_id", "mobile/precision_servo"},
    rclcpp::Parameter{"servo_to_pose.preset_schema_version", std::int64_t{3}},
    rclcpp::Parameter{"servo_to_pose.action_name", "/skills/servo_to_pose"},
    rclcpp::Parameter{
      "servo_to_pose.pause_service_name", "/skills/servo_to_pose/set_paused"},
    rclcpp::Parameter{"servo_to_pose.heartbeat_period_sec", 0.04},
    rclcpp::Parameter{"servo_to_pose.default_timeout_sec", 12.0},
    rclcpp::Parameter{"servo_to_pose.position_tolerance_m", 0.002},
    rclcpp::Parameter{"servo_to_pose.orientation_tolerance_rad", 0.03},
    rclcpp::Parameter{"servo_to_pose.stopped_velocity_tolerance", 0.015},
    rclcpp::Parameter{"servo_to_pose.required_target_samples", std::int64_t{4}},
    rclcpp::Parameter{"servo_to_pose.required_stopped_samples", std::int64_t{5}},
    rclcpp::Parameter{"servo_to_pose.halt_timeout_sec", 1.5},
    rclcpp::Parameter{"servo_to_pose.control_period_sec", 0.015},
    rclcpp::Parameter{"servo_to_pose.incoming_command_timeout_sec", 0.09},
    rclcpp::Parameter{"servo_to_pose.service_call_timeout_sec", 0.4},
    rclcpp::Parameter{"servo_to_pose.target_prepare_timeout_sec", 0.8},
    rclcpp::Parameter{"servo_to_pose.feedback_startup_timeout_sec", 0.6},
    rclcpp::Parameter{"servo_to_pose.observation_timeout_sec", 0.004},
    rclcpp::Parameter{"servo_to_pose.planning_frame", "base_link"},
    rclcpp::Parameter{"servo_to_pose.task_frame", "tool_grasp_frame"},
    rclcpp::Parameter{"servo_to_pose.servo_tip_frame", "tool_link"},
    rclcpp::Parameter{
      "servo_to_pose.joint_names", std::vector<std::string>{"joint_1", "joint_2"}},
    rclcpp::Parameter{"servo_to_pose.pose_command_topic", "/servo/pose"},
    rclcpp::Parameter{"servo_to_pose.command_type_service", "/servo/type"},
    rclcpp::Parameter{"servo_to_pose.backend_pause_service", "/servo/pause"},
    rclcpp::Parameter{"servo_to_pose.status_topic", "/servo/status"},
    rclcpp::Parameter{"servo_to_pose.joint_state_topic", "/robot/joint_states"},
    rclcpp::Parameter{"servo_to_pose.poll_period_sec", 0.006},
    rclcpp::Parameter{"servo_to_pose.current_tf_max_age_sec", 0.2},
    rclcpp::Parameter{"servo_to_pose.joint_state_max_age_sec", 0.18},
    rclcpp::Parameter{"servo_to_pose.status_max_age_sec", 0.3},
    rclcpp::Parameter{"servo_to_pose.arm_resource_id", "arm/primary"},
  };
}

TEST(MoveItServoProcessConfig, LoadsCompleteProfile)
{
  ConfigNode test_node{"complete_moveit_servo_config", complete_profile()};
  const auto config = load_moveit_servo_process_config(test_node.node());
  EXPECT_FALSE(test_node.node().has_parameter("servo_to_pose.enabled"));
  EXPECT_EQ(config.preset_identity.preset_id(), "mobile/precision_servo");
  EXPECT_EQ(config.preset_identity.schema_version(), 3U);
  EXPECT_EQ(config.action_server.action_name, "/skills/servo_to_pose");
  EXPECT_EQ(
    config.action_server.pause_service_name, "/skills/servo_to_pose/set_paused");
  EXPECT_EQ(config.action_server.heartbeat_period, 40ms);
  EXPECT_EQ(config.action_server.default_timeout, 12s);
  EXPECT_DOUBLE_EQ(config.action_server.task.state_machine.position_tolerance_m, 0.002);
  EXPECT_DOUBLE_EQ(
    config.action_server.task.state_machine.orientation_tolerance_rad, 0.03);
  EXPECT_DOUBLE_EQ(
    config.action_server.task.state_machine.stopped_velocity_tolerance, 0.015);
  EXPECT_EQ(config.action_server.task.state_machine.required_target_samples, 4U);
  EXPECT_EQ(config.action_server.task.state_machine.required_stopped_samples, 5U);
  EXPECT_EQ(config.action_server.task.state_machine.halt_timeout, 1500ms);
  EXPECT_EQ(config.action_server.task.control_period, 15ms);
  EXPECT_EQ(config.action_server.task.incoming_command_timeout, 90ms);
  EXPECT_EQ(config.action_server.task.service_call_timeout, 400ms);
  EXPECT_EQ(config.action_server.task.target_prepare_timeout, 800ms);
  EXPECT_EQ(config.action_server.task.feedback_startup_timeout, 600ms);
  EXPECT_EQ(config.action_server.task.observation_timeout, 4ms);
  EXPECT_EQ(config.adapter.planning_frame, "base_link");
  EXPECT_EQ(config.adapter.task_frame, "tool_grasp_frame");
  EXPECT_EQ(config.adapter.servo_tip_frame, "tool_link");
  EXPECT_EQ(config.adapter.joint_names, (std::vector<std::string>{"joint_1", "joint_2"}));
  EXPECT_EQ(config.adapter.endpoints.pose_command_topic, "/servo/pose");
  EXPECT_EQ(config.adapter.endpoints.command_type_service, "/servo/type");
  EXPECT_EQ(config.adapter.endpoints.pause_service, "/servo/pause");
  EXPECT_EQ(config.adapter.endpoints.status_topic, "/servo/status");
  EXPECT_EQ(config.adapter.endpoints.joint_state_topic, "/robot/joint_states");
  EXPECT_EQ(config.adapter.poll_period, 6ms);
  EXPECT_EQ(config.adapter.current_tf_max_age, 200ms);
  EXPECT_EQ(config.adapter.joint_state_max_age, 180ms);
  EXPECT_EQ(config.adapter.status_max_age, 300ms);
  EXPECT_EQ(config.arm_resource_id, "arm/primary");
}

TEST(MoveItServoProcessConfig, IdentitiesCoverPolicyAndWiring)
{
  ConfigNode test_node{"identity_moveit_servo_config", complete_profile()};
  const auto config = load_moveit_servo_process_config(test_node.node());
  const auto preset = make_moveit_servo_preset_identity(config);
  const auto deployment = make_moveit_servo_deployment_identity(config);
  EXPECT_EQ(deployment.preset_id(), "moveit/servo_process");

  auto changed_policy = config;
  changed_policy.action_server.task.state_machine.position_tolerance_m += 0.001;
  EXPECT_NE(preset, make_moveit_servo_preset_identity(changed_policy));
  EXPECT_NE(deployment, make_moveit_servo_deployment_identity(changed_policy));

  auto changed_wiring = config;
  changed_wiring.adapter.endpoints.pose_command_topic += "_alternate";
  EXPECT_EQ(preset, make_moveit_servo_preset_identity(changed_wiring));
  EXPECT_NE(deployment, make_moveit_servo_deployment_identity(changed_wiring));

  auto changed_resource = config;
  changed_resource.arm_resource_id = "arm/alternate";
  EXPECT_EQ(preset, make_moveit_servo_preset_identity(changed_resource));
  EXPECT_NE(deployment, make_moveit_servo_deployment_identity(changed_resource));
}

TEST(MoveItServoProcessConfig, RejectsMissingRobotBindingAndInvalidTiming)
{
  {
    Parameters parameters;
    ConfigNode missing_binding{"missing_moveit_servo_binding", parameters};
    EXPECT_THROW(
      (void)load_moveit_servo_process_config(missing_binding.node()),
      std::exception);
  }

  const std::vector<std::string> duration_names{
    "heartbeat_period_sec",
    "default_timeout_sec",
    "halt_timeout_sec",
    "control_period_sec",
    "incoming_command_timeout_sec",
    "service_call_timeout_sec",
    "target_prepare_timeout_sec",
    "feedback_startup_timeout_sec",
    "observation_timeout_sec",
    "poll_period_sec",
    "current_tf_max_age_sec",
    "joint_state_max_age_sec",
    "status_max_age_sec",
  };
  for (std::size_t index = 0U; index < duration_names.size(); ++index) {
    auto parameters = complete_profile();
    set_override(
      parameters,
      rclcpp::Parameter{"servo_to_pose." + duration_names[index], 0.0});
    ConfigNode invalid{
      "invalid_moveit_servo_duration_" + std::to_string(index), parameters};
    EXPECT_THROW(
      (void)load_moveit_servo_process_config(invalid.node()),
      std::invalid_argument) << duration_names[index];
  }
}

TEST(MoveItServoProcessConfig, RejectsInvalidLogicalIdsAndSampleCounts)
{
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>> invalid{
    {"servo_to_pose.preset_id", rclcpp::ParameterValue{"Bad Servo Preset"}},
    {"servo_to_pose.arm_resource_id", rclcpp::ParameterValue{"/arm/main"}},
    {"servo_to_pose.action_name", rclcpp::ParameterValue{"bad action name"}},
    {"servo_to_pose.preset_schema_version", rclcpp::ParameterValue{std::int64_t{0}}},
    {"servo_to_pose.required_target_samples", rclcpp::ParameterValue{std::int64_t{0}}},
    {"servo_to_pose.required_stopped_samples", rclcpp::ParameterValue{std::int64_t{0}}},
  };
  for (std::size_t index = 0U; index < invalid.size(); ++index) {
    auto parameters = complete_profile();
    set_override(
      parameters, rclcpp::Parameter{invalid[index].first, invalid[index].second});
    ConfigNode invalid_node{
      "invalid_moveit_servo_value_" + std::to_string(index), parameters};
    EXPECT_THROW(
      (void)load_moveit_servo_process_config(invalid_node.node()),
      std::invalid_argument) << invalid[index].first;
  }
}

}  // namespace
}  // namespace manipulation::moveit
