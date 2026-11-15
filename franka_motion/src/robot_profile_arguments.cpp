// Responsibility: Resolves the installed robot profile and preserves ROS CLI override order.
// Does not own: Profile contents, validation of node parameters, or rclcpp shutdown.

#include "franka_motion/robot_profile_arguments.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace franka_motion
{

std::vector<std::string> injectParameterFileBeforeRosOverrides(
  const int argc,
  const char * const * argv,
  const std::string & parameter_file)
{
  if (argc < 1 || argv == nullptr || argv[0] == nullptr) {
    throw std::invalid_argument("Cannot inject a parameter file without argv[0]");
  }
  if (parameter_file.empty()) {
    throw std::invalid_argument("Robot profile parameter-file path must not be empty");
  }

  std::vector<std::string> arguments;
  arguments.reserve(static_cast<std::size_t>(argc) + 4U);
  for (int index = 0; index < argc; ++index) {
    if (argv[index] == nullptr) {
      throw std::invalid_argument("ROS process arguments must not contain null entries");
    }
    arguments.emplace_back(argv[index]);
  }

  const auto application_boundary =
    std::find(arguments.begin() + 1, arguments.end(), "--");
  const auto ros_arguments =
    std::find(arguments.begin() + 1, application_boundary, "--ros-args");
  if (ros_arguments != application_boundary) {
    arguments.insert(ros_arguments + 1, {"--params-file", parameter_file});
    return arguments;
  }

  arguments.insert(
    arguments.begin() + 1,
    {"--ros-args", "--params-file", parameter_file, "--"});
  return arguments;
}

void initializeWithRobotProfile(
  const int argc,
  char ** argv,
  const rclcpp::InitOptions & init_options,
  const rclcpp::SignalHandlerOptions signal_handler_options)
{
  const auto profile_path =
    ament_index_cpp::get_package_share_directory("franka_motion") +
    "/config/manipulation/robot_profile.yaml";
  auto arguments = injectParameterFileBeforeRosOverrides(argc, argv, profile_path);
  std::vector<char *> mutable_arguments;
  mutable_arguments.reserve(arguments.size());
  for (auto & argument : arguments) {
    mutable_arguments.push_back(argument.data());
  }
  rclcpp::init(
    static_cast<int>(mutable_arguments.size()), mutable_arguments.data(),
    init_options, signal_handler_options);
}

}  // namespace franka_motion
