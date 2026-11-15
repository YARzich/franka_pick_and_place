// Responsibility: Encodes requested gripper widths and hold positions for the ros2_control command
//   topic.

#include "panda_gripper_commands.hpp"

#include <algorithm>
#include <vector>

#include "franka_motion/gripper_utils.hpp"

namespace franka_motion
{
std_msgs::msg::Float64MultiArray makePandaGripperWidthCommand(
  double width,
  std::int64_t command_joint_count,
  double minimum_joint_position,
  double maximum_joint_position)
{
  const double joint_position = std::clamp(
    widthToJoint(width), minimum_joint_position, maximum_joint_position);
  std_msgs::msg::Float64MultiArray command;
  command.data = command_joint_count == 1 ?
    std::vector<double>{joint_position} :
  std::vector<double>{joint_position, joint_position};
  return command;
}

std_msgs::msg::Float64MultiArray makePandaGripperHoldCommand(
  double left_joint,
  double right_joint,
  std::int64_t command_joint_count,
  double minimum_joint_position,
  double maximum_joint_position)
{
  const double bounded_left = std::clamp(
    left_joint, minimum_joint_position, maximum_joint_position);
  const double bounded_right = std::clamp(
    right_joint, minimum_joint_position, maximum_joint_position);
  std_msgs::msg::Float64MultiArray command;
  command.data = command_joint_count == 1 ?
    std::vector<double>{bounded_left} :
  std::vector<double>{bounded_left, bounded_right};
  return command;
}

}  // namespace franka_motion
