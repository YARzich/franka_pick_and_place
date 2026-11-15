// Responsibility: Constructs gripper ROS interfaces and exposes the package-level node factory.

#include "panda_gripper_driver.hpp"

#include <functional>
#include <memory>

#include "panda_gripper_driver_config.hpp"
#include "panda_gripper_driver_node.hpp"

namespace franka_motion::detail
{

PandaGripperDriver::PandaGripperDriver()
: Node("panda_gripper_driver")
{
  const auto config = declareAndReadPandaGripperDriverConfig(*this);
  command_topic_ = config.command_topic;
  command_joint_count_ = config.command_joint_count;
  command_joint_min_position_ = config.command_joint_min_position;
  command_joint_max_position_ = config.command_joint_max_position;
  state_timeout_ = config.goal_policy.state_timeout;
  goal_evaluator_.emplace(config.goal_policy);

  command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS());
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    config.joint_states_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PandaGripperDriver::handleJointState, this, std::placeholders::_1));
  action_server_ = rclcpp_action::create_server<GripperAction>(
    this,
    config.action_name,
    std::bind(
      &PandaGripperDriver::handleGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&PandaGripperDriver::handleCancel, this, std::placeholders::_1),
    std::bind(&PandaGripperDriver::handleAccepted, this, std::placeholders::_1));
  monitor_timer_ = create_wall_timer(
    config.monitor_period,
    std::bind(&PandaGripperDriver::monitorActiveGoal, this));

  RCLCPP_INFO(
    get_logger(),
    "Panda gripper driver готов. Action: %s, cmd: %s, state: %s",
    config.action_name.c_str(), command_topic_.c_str(),
    config.joint_states_topic.c_str());
}

}  // namespace franka_motion::detail

namespace franka_motion
{

rclcpp::Node::SharedPtr makePandaGripperDriverNode()
{
  return std::make_shared<detail::PandaGripperDriver>();
}

void preparePandaGripperDriverShutdown(
  const rclcpp::Node::SharedPtr & node) noexcept
{
  const auto driver = std::dynamic_pointer_cast<detail::PandaGripperDriver>(node);
  if (driver) {
    driver->shutdownActiveGoal();
  }
}

}  // namespace franka_motion
