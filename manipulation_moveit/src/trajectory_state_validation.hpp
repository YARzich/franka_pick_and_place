// Responsibility: Correlates a planned trajectory start with an authoritative robot state.

#ifndef TRAJECTORY_STATE_VALIDATION_HPP_
#define TRAJECTORY_STATE_VALIDATION_HPP_

#include <cstdint>
#include <string>

#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace manipulation::moveit::detail
{

enum class TrajectoryStartStateStatus : std::uint8_t
{
  kMatch,
  kMismatch,
  kInvalid,
};

struct TrajectoryStartStateCheck
{
  TrajectoryStartStateStatus status{TrajectoryStartStateStatus::kInvalid};
  std::string detail{"trajectory start state was not checked"};
};

/**
 * Compare the first waypoint with a complete authoritative robot state.
 *
 * A malformed or uncorrelatable state is distinguished from an otherwise
 * valid state that moved outside `tolerance` after planning.
 */
[[nodiscard]] TrajectoryStartStateCheck check_trajectory_start_state(
  const moveit_msgs::msg::RobotState & authoritative_state,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  double tolerance);

}  // namespace manipulation::moveit::detail

#endif  // TRAJECTORY_STATE_VALIDATION_HPP_
