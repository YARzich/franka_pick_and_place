// Responsibility: Validates MoveIt smoke settings and executes its fail-closed operation order.
// Does not own: MoveGroup, subscriptions, executor threads, or retry policy.

#include "moveit_sequence_scenario.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "franka_motion/node_parameter_utils.hpp"
#include "franka_motion/pose_geometry.hpp"
#include "manipulation/validation/frame_id.hpp"

#include "scenario_config_validation.hpp"

namespace franka_motion
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

/// Reject identifiers that cannot name a MoveIt planning group.
void requireMoveItGroupName(const std::string & value, const std::string & name)
{
  config_validation::requireNonEmpty(value, name);
  if (value.front() == '/' || std::any_of(value.begin(), value.end(), [](const char character) {
      return std::isspace(static_cast<unsigned char>(character)) != 0;
    }))
  {
    config_validation::invalidParameter(name, "must be a relative name without whitespace");
  }
}

void requireCanonicalFrame(const std::string & value, const std::string & name)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(value)) {
    config_validation::invalidParameter(name, "must be a canonical relative frame");
  }
}

/// Validate scalar ranges once and convert wall-clock seconds to steady durations.
MoveItSequenceConfig resolveMoveItSequenceConfig(
  const MoveItSequenceConfigInput & input)
{
  requireMoveItGroupName(input.planning_group, "planning_group");
  requireCanonicalFrame(input.reference_frame, "reference_frame");
  config_validation::requirePositive(input.planning_time_sec, "planning_time");
  config_validation::requirePositive(input.velocity_scaling, "velocity_scaling");
  config_validation::requirePositive(input.acceleration_scaling, "acceleration_scaling");
  if (input.velocity_scaling > 1.0) {
    config_validation::invalidParameter("velocity_scaling", "must not exceed 1");
  }
  if (input.acceleration_scaling > 1.0) {
    config_validation::invalidParameter("acceleration_scaling", "must not exceed 1");
  }

  MoveItSequenceConfig config;
  config.planning_group = input.planning_group;
  config.reference_frame = input.reference_frame;
  config.planning_time_sec = input.planning_time_sec;
  config.velocity_scaling = input.velocity_scaling;
  config.acceleration_scaling = input.acceleration_scaling;
  config.current_state_timeout = config_validation::checkedDuration(
    input.current_state_timeout_sec, "current_state_timeout_sec", false);
  config.joint_state_max_age = config_validation::checkedDuration(
    input.joint_state_max_age_sec, "joint_state_max_age", false);
  return config;
}

}  // namespace

MoveItSequenceConfig readMoveItSequenceConfig(
  const std::shared_ptr<rclcpp::Node> & node)
{
  if (!node) {
    throw std::invalid_argument("Cannot read MoveIt sequence config without a ROS node");
  }
  MoveItSequenceConfigInput input;
  input.planning_group = declareOrGetParameter(
    node, "planning_group", input.planning_group);
  input.reference_frame = declareOrGetParameter(
    node, "reference_frame", input.reference_frame);
  input.planning_time_sec = declareOrGetParameter(
    node, "planning_time", input.planning_time_sec);
  input.velocity_scaling = declareOrGetParameter(
    node, "velocity_scaling", input.velocity_scaling);
  input.acceleration_scaling = declareOrGetParameter(
    node, "acceleration_scaling", input.acceleration_scaling);
  input.current_state_timeout_sec = declareOrGetParameter(
    node, "current_state_timeout_sec", input.current_state_timeout_sec);
  input.joint_state_max_age_sec = declareOrGetParameter(
    node, "joint_state_max_age", input.joint_state_max_age_sec);
  return resolveMoveItSequenceConfig(input);
}

std::array<geometry_msgs::msg::Pose, 2> defaultMoveItSequenceTargets()
{
  return {
    makePose(0.45, 0.0, 0.35, kPi, 0.0, kPi / 4.0),
    makePose(0.35, 0.2, 0.4, kPi, 0.0, -kPi / 4.0),
  };
}

bool runMoveItSequence(
  const MoveItSequenceOperations & operations,
  const std::array<geometry_msgs::msg::Pose, 2> & targets,
  const rclcpp::Logger & logger)
{
  if (!operations.capture_initial_joints || !operations.move_to_pose ||
    !operations.move_to_joints)
  {
    throw std::invalid_argument("MoveIt sequence operations must be complete");
  }

  const auto initial_joints = operations.capture_initial_joints();
  if (!initial_joints || initial_joints->empty() ||
    !std::all_of(initial_joints->begin(), initial_joints->end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_ERROR(logger, "Не удалось получить полную начальную конфигурацию суставов");
    return false;
  }
  if (!operations.move_to_pose(targets[0], "target_pose_1")) {
    return false;
  }
  if (!operations.move_to_pose(targets[1], "target_pose_2")) {
    return false;
  }
  if (!operations.move_to_joints(*initial_joints, "initial_joint_state")) {
    RCLCPP_ERROR(logger, "Не удалось вернуться в исходную конфигурацию суставов");
    return false;
  }
  RCLCPP_INFO(logger, "MoveIt diagnostic sequence completed");
  return true;
}

}  // namespace franka_motion
