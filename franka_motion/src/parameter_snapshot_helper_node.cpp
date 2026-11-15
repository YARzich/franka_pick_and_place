// Responsibility: Builds a stable helper node and executor options in one ROS context.

#include "parameter_snapshot_helper_node.hpp"

#include <stdexcept>

#include "rclcpp/node_options.hpp"

namespace franka_motion::detail
{

rclcpp::Node::SharedPtr make_parameter_snapshot_helper_node(
  const std::string & node_name,
  const rclcpp::Context::SharedPtr & context)
{
  if (node_name.empty() || !context) {
    throw std::invalid_argument("parameter snapshot helper requires a name and context");
  }

  rclcpp::NodeOptions options;
  options.context(context);
  // Launch commonly applies __node to the executable. A local remap has higher
  // precedence and prevents this internal node from sharing the public name.
  options.arguments({"--ros-args", "--remap", "__node:=" + node_name});
  return rclcpp::Node::make_shared(node_name, options);
}

rclcpp::ExecutorOptions make_parameter_snapshot_executor_options(
  const rclcpp::Context::SharedPtr & context)
{
  if (!context) {
    throw std::invalid_argument("parameter snapshot executor requires a context");
  }
  rclcpp::ExecutorOptions options;
  options.context = context;
  return options;
}

}  // namespace franka_motion::detail
