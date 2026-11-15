// Responsibility: Shares ROS fixture, spinner and parameter builders across sync tests.

#ifndef MOVEIT_PARAMETER_SYNC_TEST_SUPPORT_HPP_
#define MOVEIT_PARAMETER_SYNC_TEST_SUPPORT_HPP_

#include <array>
#include <chrono>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "franka_motion/moveit_parameter_sync.hpp"

namespace franka_motion::test_support
{

class RclcppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

class NodeSpinner final
{
public:
  explicit NodeSpinner(
    const rclcpp::Node::SharedPtr & node,
    const std::chrono::milliseconds start_delay = std::chrono::milliseconds::zero())
  : executor_(executor_options(node))
  {
    executor_.add_node(node);
    spin_thread_ = std::thread([this, start_delay]() {
          std::this_thread::sleep_for(start_delay);
          executor_.spin();
      });
  }

  ~NodeSpinner()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  NodeSpinner(const NodeSpinner &) = delete;
  NodeSpinner & operator=(const NodeSpinner &) = delete;

private:
  static rclcpp::ExecutorOptions executor_options(
    const rclcpp::Node::SharedPtr & node)
  {
    rclcpp::ExecutorOptions options;
    options.context = node->get_node_base_interface()->get_context();
    return options;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

inline MoveItParameterSyncOptions minimal_sync_options(
  const std::string & source_node,
  const std::string & helper_node)
{
  MoveItParameterSyncOptions options;
  options.helper_node_name = helper_node;
  options.move_group_node_name = source_node;
  options.service_timeout = std::chrono::seconds(1);
  options.optional_timeout = std::chrono::seconds(1);
  options.required_names.clear();
  options.optional_names.clear();
  options.required_prefixes.clear();
  options.optional_prefixes.clear();
  return options;
}

template<typename T>
void declare_or_set_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, value);
    return;
  }
  node->set_parameter(rclcpp::Parameter(name, value));
}

inline void set_panda_acceleration_limits(
  const rclcpp::Node::SharedPtr & node,
  const double max_acceleration)
{
  static constexpr std::array<const char *, 9> kJointNames{{
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
    "panda_finger_joint1", "panda_finger_joint2"}};
  for (const auto * joint_name : kJointNames) {
    const std::string prefix =
      "robot_description_planning.joint_limits." + std::string(joint_name);
    declare_or_set_parameter(node, prefix + ".has_acceleration_limits", true);
    declare_or_set_parameter(node, prefix + ".max_acceleration", max_acceleration);
  }
}

}  // namespace franka_motion::test_support

#endif  // MOVEIT_PARAMETER_SYNC_TEST_SUPPORT_HPP_
