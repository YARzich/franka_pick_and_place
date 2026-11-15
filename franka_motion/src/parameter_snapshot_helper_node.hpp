// Responsibility: Declares context-bound ROS helper construction for parameter snapshots.

#ifndef PARAMETER_SNAPSHOT_HELPER_NODE_HPP_
#define PARAMETER_SNAPSHOT_HELPER_NODE_HPP_

#include <string>

#include "rclcpp/context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/node.hpp"

namespace franka_motion::detail
{

/// Create a temporary snapshot node without inheriting a process-wide node-name remap.
rclcpp::Node::SharedPtr make_parameter_snapshot_helper_node(
  const std::string & node_name,
  const rclcpp::Context::SharedPtr & context);

/// Bind a private executor to the same context as its snapshot helper node.
rclcpp::ExecutorOptions make_parameter_snapshot_executor_options(
  const rclcpp::Context::SharedPtr & context);

}  // namespace franka_motion::detail

#endif  // PARAMETER_SNAPSHOT_HELPER_NODE_HPP_
