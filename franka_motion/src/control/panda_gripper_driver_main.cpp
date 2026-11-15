// Responsibility: Owns process initialization, spinning and orderly shutdown of the gripper node.

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panda_gripper_driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  // Executor registration does not own the high-level Node object.
  const auto node = franka_motion::makePandaGripperDriverNode();
  executor.add_node(node);
  executor.spin();
  franka_motion::preparePandaGripperDriverShutdown(node);
  rclcpp::shutdown();
  return 0;
}
