// Responsibility: Defines the validated configuration passed into gripper driver ROS entities.

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/node.hpp"

#include "franka_motion/gripper_goal_evaluator.hpp"

namespace franka_motion
{
/// Validated runtime configuration for panda_gripper_driver.
struct PandaGripperDriverConfig
{
  /// Float64MultiArray command topic; default /panda_hand_controller/commands.
  std::string command_topic;
  /// GripperCommand action name; default /panda_gripper/gripper_cmd.
  std::string action_name;
  /// JointState feedback topic; default /joint_states.
  std::string joint_states_topic;
  /// Metre-based thresholds and steady-clock deadlines for goal evaluation.
  GripperGoalPolicy goal_policy;
  /// Wall-timer period derived from action_monitor_rate in hertz.
  std::chrono::nanoseconds monitor_period;
  /// Number of controller command values; exactly one or two.
  std::int64_t command_joint_count{2};
  /// Minimum commanded finger-joint position, in metres.
  double command_joint_min_position{0.0};
  /// Maximum commanded finger-joint position, in metres.
  double command_joint_max_position{0.04};
};

///
/// Declare every public driver parameter and return a validated snapshot.
///
/// Width, joint positions and tolerances use metres; rates use hertz; timeout
/// parameters use seconds and are converted to steady-clock durations.
PandaGripperDriverConfig declareAndReadPandaGripperDriverConfig(rclcpp::Node & node);

}  // namespace franka_motion
