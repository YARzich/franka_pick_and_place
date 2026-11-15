// Responsibility: Parses and validates paired Panda finger samples from stamped JointState
//   messages.

#pragma once

#include <cmath>
#include <optional>

#include "sensor_msgs/msg/joint_state.hpp"

namespace franka_motion
{
/// Complete finite Panda finger sample; effort remains optional in JointState.
struct PandaFingerJointSample
{
  double left_position{0.0};
  double right_position{0.0};
  std::optional<double> absolute_effort_sum;
};

///
/// Extract both named Panda finger positions independent of message ordering.
/// Returns nullopt when either position is absent, out of bounds or non-finite.
/// Effort is populated only when both corresponding values are finite.
inline std::optional<PandaFingerJointSample> parsePandaFingerJointState(
  const sensor_msgs::msg::JointState & msg)
{
  std::optional<double> left_position;
  std::optional<double> right_position;
  std::optional<double> left_effort;
  std::optional<double> right_effort;
  for (std::size_t index = 0; index < msg.name.size(); ++index) {
    if (msg.name[index] == "panda_finger_joint1" && index < msg.position.size()) {
      left_position = msg.position[index];
      if (index < msg.effort.size()) {
        left_effort = msg.effort[index];
      }
    } else if (msg.name[index] == "panda_finger_joint2" && index < msg.position.size()) {
      right_position = msg.position[index];
      if (index < msg.effort.size()) {
        right_effort = msg.effort[index];
      }
    }
  }

  if (!left_position.has_value() || !right_position.has_value() ||
    !std::isfinite(left_position.value()) || !std::isfinite(right_position.value()))
  {
    return std::nullopt;
  }

  PandaFingerJointSample sample;
  sample.left_position = left_position.value();
  sample.right_position = right_position.value();
  if (left_effort.has_value() && right_effort.has_value() &&
    std::isfinite(left_effort.value()) && std::isfinite(right_effort.value()))
  {
    sample.absolute_effort_sum =
      std::abs(left_effort.value()) + std::abs(right_effort.value());
  }
  return sample;
}
}  // namespace franka_motion
