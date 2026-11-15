// Responsibility: Exposes node creation and shutdown hooks without leaking the private driver type.

#pragma once

#include "rclcpp/node.hpp"

namespace franka_motion
{
///
/// Construct the standalone Panda gripper ROS adapter.
///
/// Mutable action and feedback state relies on the SingleThreadedExecutor in
/// the package main. This factory is not a composable/thread-safe node API;
/// component support requires explicit callback-group or locking design first.
rclcpp::Node::SharedPtr makePandaGripperDriverNode();

/// Finalize process-owned action state after the executor stops.
void preparePandaGripperDriverShutdown(
  const rclcpp::Node::SharedPtr & node) noexcept;

}  // namespace franka_motion
