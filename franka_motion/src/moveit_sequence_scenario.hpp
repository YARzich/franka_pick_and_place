// Responsibility: Defines the validated policy and ordered operations of the MoveIt smoke demo.
// Does not own: ROS process lifecycle, MoveGroup construction, or action transport.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/logger.hpp"

namespace rclcpp
{
class Node;
}

namespace franka_motion
{

/// Raw ROS parameters for the low-level two-pose MoveIt diagnostic.
struct MoveItSequenceConfigInput
{
  std::string planning_group{"panda_arm"};
  std::string reference_frame{"panda_link0"};
  double planning_time_sec{5.0};
  double velocity_scaling{0.2};
  double acceleration_scaling{0.2};
  double current_state_timeout_sec{10.0};
  double joint_state_max_age_sec{0.5};
};

/// Validated settings with steady-clock state deadlines.
struct MoveItSequenceConfig
{
  std::string planning_group;
  std::string reference_frame;
  double planning_time_sec{5.0};
  double velocity_scaling{0.2};
  double acceleration_scaling{0.2};
  std::chrono::steady_clock::duration current_state_timeout{};
  std::chrono::steady_clock::duration joint_state_max_age{};
};

/// Side-effect boundary supplied by the MoveGroup adapter in the entrypoint.
struct MoveItSequenceOperations
{
  std::function<std::optional<std::vector<double>>()> capture_initial_joints;
  std::function<bool(const geometry_msgs::msg::Pose &, const std::string &)> move_to_pose;
  std::function<bool(const std::vector<double> &, const std::string &)> move_to_joints;
};

/// Declare and validate only the supported diagnostic parameters.
MoveItSequenceConfig readMoveItSequenceConfig(
  const std::shared_ptr<rclcpp::Node> & node);

/// Return the two historical Panda targets in their execution order.
std::array<geometry_msgs::msg::Pose, 2> defaultMoveItSequenceTargets();

/// Execute pose1, pose2, then the captured initial joints; stop on any failure.
bool runMoveItSequence(
  const MoveItSequenceOperations & operations,
  const std::array<geometry_msgs::msg::Pose, 2> & targets,
  const rclcpp::Logger & logger);

}  // namespace franka_motion
