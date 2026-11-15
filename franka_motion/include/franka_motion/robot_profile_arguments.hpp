// Responsibility: Loads the installed robot profile before process-specific ROS overrides.
// Does not own: Robot values, node configuration policy, or process lifecycle after rclcpp init.

#ifndef FRANKA_MOTION__ROBOT_PROFILE_ARGUMENTS_HPP_
#define FRANKA_MOTION__ROBOT_PROFILE_ARGUMENTS_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace franka_motion
{

/// Insert one parameter file before the first user ROS argument.
///
/// The returned strings own their storage. If no `--ros-args` block exists, a
/// self-contained block is inserted after argv[0] and closed before positional
/// application arguments. Later user `--params-file` and `-p` options therefore
/// retain normal last-override-wins semantics.
std::vector<std::string> injectParameterFileBeforeRosOverrides(
  int argc,
  const char * const * argv,
  const std::string & parameter_file);

/// Initialize rclcpp with the installed `franka_motion` robot profile.
void initializeWithRobotProfile(
  int argc,
  char ** argv,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions{},
  rclcpp::SignalHandlerOptions signal_handler_options =
  rclcpp::SignalHandlerOptions::All);

}  // namespace franka_motion

#endif  // FRANKA_MOTION__ROBOT_PROFILE_ARGUMENTS_HPP_
