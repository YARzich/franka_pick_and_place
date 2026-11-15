// Responsibility: Validates Servo ROS wiring and maps stable MoveIt status codes to safety states.

#include "manipulation_moveit/servo/moveit_servo_ros_adapter_types.hpp"

#include <cctype>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "moveit_msgs/msg/servo_status.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

#include "manipulation/validation/frame_id.hpp"

namespace manipulation::moveit
{
namespace
{

bool is_valid_joint_name(const std::string & name) noexcept
{
  if (name.empty()) {
    return false;
  }
  for (const char raw_character : name) {
    const auto character = static_cast<unsigned char>(raw_character);
    if (std::isspace(character) != 0 || std::iscntrl(character) != 0) {
      return false;
    }
  }
  return true;
}

std::optional<std::string> validate_endpoint(
  const std::string & name, const char * label, const bool is_service)
{
  if (name.empty()) {
    return std::string{label} + " must be non-empty";
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      name, "moveit_servo_adapter", "/", is_service);
  } catch (const std::exception & error) {
    return std::string{label} + " is not a valid ROS name: " + error.what();
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> validate_moveit_servo_ros_adapter_config(
  const MoveItServoRosAdapterConfig & config)
{
  const std::pair<const char *, const std::string *> frames[] = {
    {"planning_frame", &config.planning_frame},
    {"task_frame", &config.task_frame},
    {"servo_tip_frame", &config.servo_tip_frame},
  };
  for (const auto & [label, frame] : frames) {
    if (!manipulation::validation::is_canonical_relative_frame_id(*frame)) {
      return std::string{label} + " must be a canonical relative TF frame";
    }
  }

  if (config.joint_names.empty()) {
    return "joint_names must contain at least one controlled joint";
  }
  std::set<std::string> unique_joints;
  for (const auto & joint_name : config.joint_names) {
    if (!is_valid_joint_name(joint_name)) {
      return "joint_names must contain non-empty names without whitespace";
    }
    if (!unique_joints.emplace(joint_name).second) {
      return "joint_names must not contain duplicates";
    }
  }

  const struct
  {
    const std::string * name;
    const char * label;
    bool is_service;
  } endpoints[] = {
    {&config.endpoints.pose_command_topic, "pose_command_topic", false},
    {&config.endpoints.command_type_service, "command_type_service", true},
    {&config.endpoints.pause_service, "pause_service", true},
    {&config.endpoints.status_topic, "status_topic", false},
    {&config.endpoints.joint_state_topic, "joint_state_topic", false},
  };
  for (const auto & endpoint : endpoints) {
    if (auto error = validate_endpoint(
        *endpoint.name, endpoint.label, endpoint.is_service))
    {
      return error;
    }
  }

  const struct
  {
    std::chrono::steady_clock::duration value;
    const char * label;
  } durations[] = {
    {config.poll_period, "poll_period"},
    {config.current_tf_max_age, "current_tf_max_age"},
    {config.joint_state_max_age, "joint_state_max_age"},
    {config.status_max_age, "status_max_age"},
  };
  for (const auto & duration : durations) {
    if (duration.value <= std::chrono::steady_clock::duration::zero()) {
      return std::string{duration.label} + " must be positive";
    }
  }
  return std::nullopt;
}

MoveItServoRosResult MoveItServoRosResult::success(std::string detail)
{
  return MoveItServoRosResult{MoveItServoRosError::kNone, std::move(detail)};
}

MoveItServoRosResult MoveItServoRosResult::failure(
  const MoveItServoRosError error, std::string detail)
{
  if (error == MoveItServoRosError::kNone) {
    throw std::invalid_argument("failed Servo ROS result requires a failure code");
  }
  return MoveItServoRosResult{error, std::move(detail)};
}

MoveItServoRosResult::MoveItServoRosResult(
  const MoveItServoRosError error, std::string detail)
: error_(error), detail_(std::move(detail))
{
}

bool MoveItServoRosResult::succeeded() const noexcept
{
  return error_ == MoveItServoRosError::kNone;
}

MoveItServoRosError MoveItServoRosResult::error() const noexcept
{
  return error_;
}

const std::string & MoveItServoRosResult::detail() const noexcept
{
  return detail_;
}

bool is_known_moveit_servo_status(const std::int8_t code) noexcept
{
  using Status = moveit_msgs::msg::ServoStatus;
  switch (code) {
    case Status::INVALID:
    case Status::NO_WARNING:
    case Status::DECELERATE_FOR_APPROACHING_SINGULARITY:
    case Status::HALT_FOR_SINGULARITY:
    case Status::DECELERATE_FOR_LEAVING_SINGULARITY:
    case Status::DECELERATE_FOR_COLLISION:
    case Status::HALT_FOR_COLLISION:
    case Status::JOINT_BOUND:
      return true;
    default:
      return false;
  }
}

manipulation::servo::ServoSafetyStatus classify_moveit_servo_status(
  const std::int8_t code) noexcept
{
  using Safety = manipulation::servo::ServoSafetyStatus;
  using Status = moveit_msgs::msg::ServoStatus;
  switch (code) {
    case Status::NO_WARNING:
      return Safety::kNominal;
    case Status::DECELERATE_FOR_APPROACHING_SINGULARITY:
    case Status::DECELERATE_FOR_LEAVING_SINGULARITY:
    case Status::DECELERATE_FOR_COLLISION:
      return Safety::kAdvisory;
    case Status::INVALID:
    case Status::HALT_FOR_SINGULARITY:
    case Status::HALT_FOR_COLLISION:
    case Status::JOINT_BOUND:
    default:
      return Safety::kStopRequired;
  }
}

}  // namespace manipulation::moveit
