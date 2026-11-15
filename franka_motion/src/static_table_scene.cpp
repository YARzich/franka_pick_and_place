// Responsibility: Routes the historical table-scene executable to the coordinated writer.
// Boundary: execv preserves arguments, signals and the implementation's exit status.

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

bool resolve_use_coordinator(int argc, char ** argv)
{
  // Let rclcpp resolve CLI and parameter-file selectors exactly as it would
  // for the target node; token scanning would silently mishandle YAML files.
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.context(context);
    options.enable_rosout(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    bool use_coordinator = true;
    {
      const auto parameter_reader =
        std::make_shared<rclcpp::Node>("static_table_scene", options);
      use_coordinator = parameter_reader->declare_parameter<bool>(
        "use_scene_coordinator", true);
    }
    context->shutdown("static_table_scene route resolved");
    return use_coordinator;
  } catch (...) {
    context->shutdown("static_table_scene route resolution failed");
    throw;
  }
}

std::filesystem::path implementation_path()
{
  const std::string package = "franka_motion";
  const std::filesystem::path prefix =
    ament_index_cpp::get_package_prefix(package);
  return prefix / "libexec" / package / "static_table_scene_coordinated";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (!resolve_use_coordinator(argc, argv)) {
      std::fprintf(
        stderr,
        "static_table_scene: the legacy direct PlanningScene writer was removed; "
        "use_scene_coordinator=false is no longer supported\n");
      return 1;
    }
  } catch (const std::exception & error) {
    std::fprintf(stderr, "franka_motion facade parameter error: %s\n", error.what());
    return 1;
  }

  try {
    const std::filesystem::path executable = implementation_path();
    ::execv(executable.c_str(), argv);
    std::fprintf(
      stderr, "franka_motion facade could not exec %s: errno=%d\n",
      executable.c_str(), errno);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "franka_motion facade lookup failed: %s\n", error.what());
  }
  return 127;
}
