// Responsibility: Loads and validates one required MoveIt Servo process snapshot.

#include "manipulation_moveit/config/moveit_servo_process_config.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/skill/logical_id.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

namespace manipulation::moveit
{
namespace
{

constexpr char kPrefix[] = "servo_to_pose.";

std::string parameter_name(const std::string & suffix)
{
  return std::string{kPrefix} + suffix;
}

std::chrono::steady_clock::duration positive_duration_parameter(
  rclcpp::Node & node,
  const std::string & suffix,
  const double default_seconds)
{
  const auto name = parameter_name(suffix);
  const double seconds = node.declare_parameter<double>(name, default_seconds);
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{seconds});
}

std::uint32_t positive_uint32_parameter(
  rclcpp::Node & node,
  const std::string & suffix,
  const std::int64_t default_value)
{
  const auto name = parameter_name(suffix);
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0 ||
    static_cast<std::uint64_t>(value) > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::invalid_argument(name + " must fit the positive uint32 range");
  }
  return static_cast<std::uint32_t>(value);
}

std::string logical_id_parameter(
  rclcpp::Node & node,
  const std::string & suffix,
  const std::string & default_value)
{
  const auto name = parameter_name(suffix);
  auto value = node.declare_parameter<std::string>(name, default_value);
  if (!manipulation::skill::is_canonical_logical_id(value)) {
    throw std::invalid_argument(name + " must be a canonical logical ID");
  }
  return value;
}

void validate_endpoint(
  const std::string & value,
  const std::string & suffix,
  const bool is_service)
{
  if (value.empty()) {
    throw std::invalid_argument(parameter_name(suffix) + " must be non-empty");
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      value, "moveit_servo_task_server", "/", is_service);
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            parameter_name(suffix) + " is not a valid ROS name: " + error.what());
  }
}

ServoToPoseTaskConfig load_task_config(rclcpp::Node & node)
{
  ServoToPoseTaskConfig config;
  config.state_machine.position_tolerance_m = node.declare_parameter<double>(
    parameter_name("position_tolerance_m"),
    config.state_machine.position_tolerance_m);
  config.state_machine.orientation_tolerance_rad = node.declare_parameter<double>(
    parameter_name("orientation_tolerance_rad"),
    config.state_machine.orientation_tolerance_rad);
  config.state_machine.stopped_velocity_tolerance = node.declare_parameter<double>(
    parameter_name("stopped_velocity_tolerance"),
    config.state_machine.stopped_velocity_tolerance);
  config.state_machine.required_target_samples = positive_uint32_parameter(
    node, "required_target_samples", config.state_machine.required_target_samples);
  config.state_machine.required_stopped_samples = positive_uint32_parameter(
    node, "required_stopped_samples", config.state_machine.required_stopped_samples);
  config.state_machine.halt_timeout = positive_duration_parameter(
    node, "halt_timeout_sec", 2.0);
  config.control_period = positive_duration_parameter(node, "control_period_sec", 0.02);
  config.incoming_command_timeout = positive_duration_parameter(
    node, "incoming_command_timeout_sec", 0.1);
  config.service_call_timeout = positive_duration_parameter(
    node, "service_call_timeout_sec", 0.5);
  config.target_prepare_timeout = positive_duration_parameter(
    node, "target_prepare_timeout_sec", 1.0);
  config.feedback_startup_timeout = positive_duration_parameter(
    node, "feedback_startup_timeout_sec", 0.5);
  config.observation_timeout = positive_duration_parameter(
    node, "observation_timeout_sec", 0.005);
  if (const auto error = validate_servo_to_pose_task_config(config)) {
    throw std::invalid_argument("invalid ServoToPose task policy: " + *error);
  }
  return config;
}

MoveItServoRosAdapterConfig load_adapter_config(rclcpp::Node & node)
{
  MoveItServoRosAdapterConfig config;
  config.planning_frame = node.declare_parameter<std::string>(
    parameter_name("planning_frame"));
  config.task_frame = node.declare_parameter<std::string>(
    parameter_name("task_frame"));
  config.servo_tip_frame = node.declare_parameter<std::string>(
    parameter_name("servo_tip_frame"));
  config.joint_names = node.declare_parameter<std::vector<std::string>>(
    parameter_name("joint_names"));
  config.endpoints.pose_command_topic = node.declare_parameter<std::string>(
    parameter_name("pose_command_topic"), config.endpoints.pose_command_topic);
  config.endpoints.command_type_service = node.declare_parameter<std::string>(
    parameter_name("command_type_service"), config.endpoints.command_type_service);
  config.endpoints.pause_service = node.declare_parameter<std::string>(
    parameter_name("backend_pause_service"), config.endpoints.pause_service);
  config.endpoints.status_topic = node.declare_parameter<std::string>(
    parameter_name("status_topic"), config.endpoints.status_topic);
  config.endpoints.joint_state_topic = node.declare_parameter<std::string>(
    parameter_name("joint_state_topic"), config.endpoints.joint_state_topic);
  config.poll_period = positive_duration_parameter(node, "poll_period_sec", 0.005);
  config.current_tf_max_age = positive_duration_parameter(
    node, "current_tf_max_age_sec", 0.25);
  config.joint_state_max_age = positive_duration_parameter(
    node, "joint_state_max_age_sec", 0.25);
  config.status_max_age = positive_duration_parameter(
    node, "status_max_age_sec", 0.5);
  if (const auto error = validate_moveit_servo_ros_adapter_config(config)) {
    throw std::invalid_argument("invalid MoveIt Servo ROS adapter: " + *error);
  }
  return config;
}

}  // namespace

LoadedMoveItServoProcessConfig load_moveit_servo_process_config(
  rclcpp::Node & node)
{
  ServoToPoseActionServerConfig action_server;
  action_server.action_name = node.declare_parameter<std::string>(
    parameter_name("action_name"), action_server.action_name);
  action_server.pause_service_name = node.declare_parameter<std::string>(
    parameter_name("pause_service_name"), action_server.pause_service_name);
  action_server.heartbeat_period = positive_duration_parameter(
    node, "heartbeat_period_sec", 0.025);
  action_server.default_timeout = positive_duration_parameter(
    node, "default_timeout_sec", 30.0);
  action_server.task = load_task_config(node);
  validate_endpoint(action_server.action_name, "action_name", false);
  validate_endpoint(action_server.pause_service_name, "pause_service_name", true);

  auto adapter = load_adapter_config(node);
  const auto preset_id = logical_id_parameter(
    node, "preset_id", "servo_to_pose/default");
  const auto schema_version = positive_uint32_parameter(
    node, "preset_schema_version", 1);
  const auto arm_resource_id = logical_id_parameter(
    node, "arm_resource_id", "arm/main");

  LoadedMoveItServoProcessConfig loaded{
    manipulation::skill::PresetIdentity::from_canonical_content(
      preset_id, schema_version, "pending"),
    std::move(action_server),
    std::move(adapter),
    arm_resource_id};
  loaded.preset_identity = make_moveit_servo_preset_identity(loaded);
  return loaded;
}

}  // namespace manipulation::moveit
