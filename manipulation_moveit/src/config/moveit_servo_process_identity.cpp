// Responsibility: Fingerprints ServoToPose presets and process deployments.

#include "manipulation_moveit/config/moveit_servo_process_config.hpp"

#include <chrono>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string_view>
#include <vector>

namespace manipulation::moveit
{
namespace
{

void append_string(
  std::ostringstream & output,
  const std::string_view name,
  const std::string_view value)
{
  output << name << '=' << value.size() << ':' << value << '\n';
}

void append_string_vector(
  std::ostringstream & output,
  const std::string_view name,
  const std::vector<std::string> & values)
{
  output << name << ".size=" << values.size() << '\n';
  for (const auto & value : values) {
    append_string(output, name, value);
  }
}

void append_double(
  std::ostringstream & output,
  const std::string_view name,
  const double value)
{
  output << name << '=' << std::hexfloat << value << std::defaultfloat << '\n';
}

void append_duration(
  std::ostringstream & output,
  const std::string_view name,
  const std::chrono::steady_clock::duration value)
{
  using Period = std::chrono::steady_clock::duration::period;
  output << name << '=' << value.count() << '@' << Period::num << '/' << Period::den << '\n';
}

void append_identity(
  std::ostringstream & output,
  const std::string_view name,
  const manipulation::skill::PresetIdentity & identity)
{
  append_string(output, std::string{name} + ".id", identity.preset_id());
  output << name << ".schema=" << identity.schema_version() << '\n';
  append_string(output, std::string{name} + ".digest", identity.canonical_digest());
}

std::string canonical_servo_content(const LoadedMoveItServoProcessConfig & config)
{
  const auto & task = config.action_server.task;
  const auto & machine = task.state_machine;
  const auto & adapter = config.adapter;

  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "manipulation_moveit.LoadedMoveItServoProcessConfig\n";
  append_double(output, "position_tolerance_m", machine.position_tolerance_m);
  append_double(output, "orientation_tolerance_rad", machine.orientation_tolerance_rad);
  append_double(
    output, "stopped_velocity_tolerance", machine.stopped_velocity_tolerance);
  output << "required_target_samples=" << machine.required_target_samples << '\n';
  output << "required_stopped_samples=" << machine.required_stopped_samples << '\n';
  append_duration(output, "halt_timeout", machine.halt_timeout);
  append_duration(output, "control_period", task.control_period);
  append_duration(output, "incoming_command_timeout", task.incoming_command_timeout);
  append_duration(output, "service_call_timeout", task.service_call_timeout);
  append_duration(output, "target_prepare_timeout", task.target_prepare_timeout);
  append_duration(output, "feedback_startup_timeout", task.feedback_startup_timeout);
  append_duration(output, "observation_timeout", task.observation_timeout);
  append_string(output, "planning_frame", adapter.planning_frame);
  append_string(output, "task_frame", adapter.task_frame);
  append_string(output, "servo_tip_frame", adapter.servo_tip_frame);
  append_string_vector(output, "joint_names", adapter.joint_names);
  return output.str();
}

std::string canonical_servo_deployment_content(
  const LoadedMoveItServoProcessConfig & config)
{
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "manipulation_moveit.MoveItServoDeployment\n";
  append_identity(output, "preset", make_moveit_servo_preset_identity(config));
  append_string(output, "action_name", config.action_server.action_name);
  append_string(
    output, "pause_service_name", config.action_server.pause_service_name);
  append_duration(
    output, "heartbeat_period", config.action_server.heartbeat_period);
  append_duration(output, "default_timeout", config.action_server.default_timeout);
  append_string(
    output, "pose_command_topic", config.adapter.endpoints.pose_command_topic);
  append_string(
    output, "command_type_service", config.adapter.endpoints.command_type_service);
  append_string(
    output, "backend_pause_service", config.adapter.endpoints.pause_service);
  append_string(output, "status_topic", config.adapter.endpoints.status_topic);
  append_string(
    output, "joint_state_topic", config.adapter.endpoints.joint_state_topic);
  append_duration(output, "poll_period", config.adapter.poll_period);
  append_duration(
    output, "current_tf_max_age", config.adapter.current_tf_max_age);
  append_duration(
    output, "joint_state_max_age", config.adapter.joint_state_max_age);
  append_duration(output, "status_max_age", config.adapter.status_max_age);
  append_string(output, "arm_resource_id", config.arm_resource_id);
  return output.str();
}

}  // namespace

manipulation::skill::PresetIdentity make_moveit_servo_preset_identity(
  const LoadedMoveItServoProcessConfig & config)
{
  return manipulation::skill::PresetIdentity::from_canonical_content(
    config.preset_identity.preset_id(),
    config.preset_identity.schema_version(),
    canonical_servo_content(config));
}

manipulation::skill::PresetIdentity make_moveit_servo_deployment_identity(
  const LoadedMoveItServoProcessConfig & config)
{
  return manipulation::skill::PresetIdentity::from_canonical_content(
    "moveit/servo_process",
    1U,
    canonical_servo_deployment_content(config));
}

}  // namespace manipulation::moveit
