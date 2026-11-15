// Responsibility: Defines fail-closed MoveIt snapshot preparation for the Franka server.

#ifndef WORKFLOW__FRANKA_TRANSFER_SEQUENCE_STARTUP_HPP_
#define WORKFLOW__FRANKA_TRANSFER_SEQUENCE_STARTUP_HPP_

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "franka_motion/moveit_parameter_sync.hpp"

namespace franka_motion
{

/// Return the bounded MoveIt parameter policy used before publishing sequence actions.
[[nodiscard]] MoveItParameterSyncOptions
make_franka_transfer_sequence_moveit_sync_options();

/** Copy, validate, and republish the immutable Panda MoveIt parameter snapshot. */
bool prepare_franka_transfer_sequence_moveit_snapshot(
  const rclcpp::Node::SharedPtr & node,
  std::function<bool()> stop_requested = {});

}  // namespace franka_motion

#endif  // WORKFLOW__FRANKA_TRANSFER_SEQUENCE_STARTUP_HPP_
