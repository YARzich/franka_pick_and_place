// Responsibility: Starts the standalone ServoToPose server and its ROS executor.

#include <exception>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/config/moveit_servo_process_config.hpp"
#include "manipulation_moveit/moveit_servo_task_host.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("moveit_servo_task_server");
    auto config = manipulation::moveit::load_moveit_servo_process_config(*node);
    manipulation::moveit::MoveItServoTaskHost host(*node, std::move(config));
    (void)host;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("moveit_servo_task_server"), "%s", error.what());
    exit_code = 1;
  } catch (...) {
    RCLCPP_FATAL(
      rclcpp::get_logger("moveit_servo_task_server"),
      "unknown exception during server startup");
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
