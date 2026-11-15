// Responsibility: Runs the standalone PlanningScene coordinator process with fatal error reporting.

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "planning_scene_coordinator_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int return_code = 0;
  try {
    const auto node =
      std::make_shared<manipulation::scene::PlanningSceneCoordinatorNode>();

    // Scene-I/O callbacks have a mutually-exclusive admission group. A second
    // thread can therefore always process the MoveIt response, while the other
    // workers keep metadata, TF and diagnostics responsive.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("planning_scene_coordinator"), "%s", error.what());
    return_code = 1;
  }
  rclcpp::shutdown();
  return return_code;
}
