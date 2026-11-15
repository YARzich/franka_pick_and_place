// Responsibility: Declares, reads and validates all runtime parameters of the gripper driver.

#include "panda_gripper_driver_config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

#include "rclcpp/expand_topic_or_service_name.hpp"

namespace franka_motion
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

SteadyClock::duration durationFromSeconds(double seconds)
{
  const double maximum_seconds =
    std::chrono::duration<double>(SteadyClock::duration::max()).count();
  if (!std::isfinite(seconds) || seconds < 0.0 || seconds > maximum_seconds) {
    throw std::invalid_argument("timeout не представим в steady-clock duration");
  }
  return std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(seconds));
}

double finiteParameterAtLeast(
  const rclcpp::Node & node,
  const std::string & name,
  double minimum)
{
  const double value = node.get_parameter(name).as_double();
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " должен быть конечным числом");
  }
  return std::max(minimum, value);
}

void validateRosTopicName(
  const rclcpp::Node & node,
  const std::string & parameter_name,
  const std::string & value)
{
  if (value.empty()) {
    throw std::invalid_argument(parameter_name + " не должен быть пустым");
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      value, node.get_name(), node.get_namespace(), false);
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            parameter_name + " содержит некорректное ROS-имя: " +
            error.what());
  }
}
}  // namespace

PandaGripperDriverConfig declareAndReadPandaGripperDriverConfig(rclcpp::Node & node)
{
  node.declare_parameter<std::string>("command_topic", "/panda_hand_controller/commands");
  node.declare_parameter<std::string>("action_name", "/panda_gripper/gripper_cmd");
  node.declare_parameter<std::string>("joint_states_topic", "/joint_states");
  node.declare_parameter<double>("goal_tolerance", 0.004);
  node.declare_parameter<double>("state_timeout_sec", 0.5);
  node.declare_parameter<double>("stall_timeout_sec", 0.5);
  node.declare_parameter<double>("stall_width_threshold", 0.0005);
  node.declare_parameter<double>("goal_timeout_sec", 3.0);
  node.declare_parameter<double>("action_monitor_rate", 20.0);
  node.declare_parameter<bool>("allow_stalling", true);
  node.declare_parameter<int>("command_joint_count", 2);
  node.declare_parameter<double>("command_joint_min_position", 0.0);
  node.declare_parameter<double>("command_joint_max_position", 0.04);
  node.declare_parameter<double>("finger_symmetry_tolerance", 0.01);
  node.declare_parameter<double>("stall_finger_reverse_tolerance", 0.002);

  PandaGripperDriverConfig config;
  config.command_topic = node.get_parameter("command_topic").as_string();
  config.action_name = node.get_parameter("action_name").as_string();
  config.joint_states_topic = node.get_parameter("joint_states_topic").as_string();
  validateRosTopicName(node, "command_topic", config.command_topic);
  validateRosTopicName(node, "action_name", config.action_name);
  validateRosTopicName(node, "joint_states_topic", config.joint_states_topic);
  config.goal_policy.goal_tolerance = finiteParameterAtLeast(node, "goal_tolerance", 0.0);
  config.goal_policy.state_timeout = durationFromSeconds(
    finiteParameterAtLeast(node, "state_timeout_sec", 0.05));
  config.goal_policy.stall_timeout = durationFromSeconds(
    finiteParameterAtLeast(node, "stall_timeout_sec", 0.05));
  config.goal_policy.goal_timeout = durationFromSeconds(
    finiteParameterAtLeast(node, "goal_timeout_sec", 0.1));
  config.goal_policy.movement_threshold = finiteParameterAtLeast(
    node, "stall_width_threshold", 0.0);
  config.goal_policy.allow_stalling = node.get_parameter("allow_stalling").as_bool();

  const double monitor_rate = finiteParameterAtLeast(node, "action_monitor_rate", 1.0);
  config.monitor_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / monitor_rate));
  if (config.monitor_period <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument(
            "action_monitor_rate слишком велик для steady-clock timer");
  }

  config.command_joint_count = node.get_parameter("command_joint_count").as_int();
  if (config.command_joint_count != 1 && config.command_joint_count != 2) {
    throw std::invalid_argument("command_joint_count должен быть равен 1 или 2");
  }

  config.command_joint_min_position =
    node.get_parameter("command_joint_min_position").as_double();
  config.command_joint_max_position =
    node.get_parameter("command_joint_max_position").as_double();
  if (!std::isfinite(config.command_joint_min_position) ||
    !std::isfinite(config.command_joint_max_position) ||
    config.command_joint_min_position < 0.0 ||
    config.command_joint_max_position > 0.04 ||
    config.command_joint_min_position >= config.command_joint_max_position)
  {
    throw std::invalid_argument(
            "границы joint-команды должны удовлетворять "
            "0 <= min < max <= 0.04");
  }

  config.goal_policy.finger_symmetry_tolerance =
    node.get_parameter("finger_symmetry_tolerance").as_double();
  if (!std::isfinite(config.goal_policy.finger_symmetry_tolerance) ||
    config.goal_policy.finger_symmetry_tolerance < 0.0)
  {
    throw std::invalid_argument(
            "finger_symmetry_tolerance должен быть конечным "
            "и неотрицательным");
  }

  config.goal_policy.maximum_reverse_motion =
    node.get_parameter("stall_finger_reverse_tolerance").as_double();
  if (!std::isfinite(config.goal_policy.maximum_reverse_motion) ||
    config.goal_policy.maximum_reverse_motion < 0.0)
  {
    throw std::invalid_argument(
            "stall_finger_reverse_tolerance должен быть конечным "
            "и неотрицательным");
  }

  return config;
}

}  // namespace franka_motion
