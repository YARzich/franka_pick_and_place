// Responsibility: Start and spin the single Franka V2 transfer composition root.

#include <exception>
#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "franka_motion/robot_profile_arguments.hpp"

#include "workflow/franka_transfer_sequence_startup.hpp"
#include "workflow_v2/composition.hpp"
#include "workflow_v2/process_config.hpp"

namespace
{

constexpr char kNodeName[] = "franka_transfer_object_sequence_server";

}  // namespace

int main(int argc, char ** argv)
{
  franka_motion::initializeWithRobotProfile(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>(kNodeName);
    auto config = franka_motion::workflow_v2::load_v2_process_config(node);
    if (!franka_motion::prepare_franka_transfer_sequence_moveit_snapshot(
        node, []() {return !rclcpp::ok();}))
    {
      throw std::runtime_error("failed to prepare the immutable MoveIt parameter snapshot");
    }

    franka_motion::workflow_v2::FrankaTransferWorkflowV2 workflow(
      node, std::move(config));
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "%s", error.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "unknown V2 workflow startup failure");
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
