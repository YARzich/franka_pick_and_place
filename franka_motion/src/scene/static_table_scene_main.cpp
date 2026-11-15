// Responsibility: Runs the coordinated one-shot static table publisher executable.

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "static_table_scene_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int return_code = 1;
  try {
    const auto node =
      std::make_shared<franka_motion::scene::StaticTableSceneNode>();
    return_code = node->apply() ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("static_table_scene"), "%s", error.what());
  }
  rclcpp::shutdown();
  return return_code;
}
