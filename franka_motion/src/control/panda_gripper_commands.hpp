// Responsibility: Declares conversion from physical gripper widths to controller command messages.

#pragma once

#include <cstdint>

#include "std_msgs/msg/float64_multi_array.hpp"

namespace franka_motion
{
/// Encode a full gripper width as one or two bounded finger-joint commands.
std_msgs::msg::Float64MultiArray makePandaGripperWidthCommand(
  double width,
  std::int64_t command_joint_count,
  double minimum_joint_position,
  double maximum_joint_position);

/// Encode measured finger positions as a bounded controller hold command.
std_msgs::msg::Float64MultiArray makePandaGripperHoldCommand(
  double left_joint,
  double right_joint,
  std::int64_t command_joint_count,
  double minimum_joint_position,
  double maximum_joint_position);

}  // namespace franka_motion
