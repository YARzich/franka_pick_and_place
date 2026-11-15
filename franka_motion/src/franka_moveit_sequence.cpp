// Responsibility: Wires the low-level Panda MoveIt diagnostic to shared bounded helpers.
// Does not own: MoveIt servers, controller lifecycle, or task-level manipulation actions.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "franka_motion/executor_spinner.hpp"
#include "franka_motion/joint_state_monitor.hpp"
#include "franka_motion/moveit_execution_utils.hpp"
#include "franka_motion/moveit_parameter_sync.hpp"
#include "franka_motion/robot_profile_arguments.hpp"

#include "moveit_sequence_scenario.hpp"

namespace
{

/// Capture one complete, finite active-group state after fresh joint feedback.
std::optional<std::vector<double>> captureInitialJoints(
  moveit::planning_interface::MoveGroupInterface & move_group,
  const franka_motion::FrankaJointStateMonitor & joint_state_monitor,
  const franka_motion::MoveItSequenceConfig & config,
  const rclcpp::Logger & logger)
{
  if (!franka_motion::waitForCurrentState(
      move_group, joint_state_monitor, logger, "initial_joint_state",
      config.current_state_timeout, config.joint_state_max_age,
      []() {return !rclcpp::ok();}))
  {
    return std::nullopt;
  }
  const auto state = move_group.getCurrentState(0.0);
  if (!state) {
    return std::nullopt;
  }
  const auto * group = state->getJointModelGroup(config.planning_group);
  if (group == nullptr) {
    return std::nullopt;
  }
  std::vector<double> joints;
  state->copyJointGroupPositions(group, joints);
  if (joints.size() != group->getVariableCount() ||
    !std::all_of(joints.begin(), joints.end(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    return std::nullopt;
  }
  return joints;
}

/// Assemble the diagnostic node, MoveGroup adapter and pure sequence operations.
int runDiagnostic()
{
  auto node = rclcpp::Node::make_shared("franka_moveit_sequence_node");
  const auto config = franka_motion::readMoveItSequenceConfig(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  franka_motion::ExecutorSpinner spinner(executor);

  franka_motion::MoveItParameterSyncOptions sync_options;
  sync_options.helper_node_name = "franka_moveit_sequence_param_helper";
  sync_options.service_unavailable_message =
    "Сервис параметров move_group недоступен, убедитесь что MoveIt запущен";
  if (!franka_motion::copy_moveit_parameters_from_move_group(node, sync_options)) {
    return rclcpp::ok() ? 1 : 130;
  }

  moveit::planning_interface::MoveGroupInterface move_group(
    node, config.planning_group);
  move_group.setPlanningTime(config.planning_time_sec);
  move_group.setPoseReferenceFrame(config.reference_frame);
  move_group.setMaxVelocityScalingFactor(config.velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(config.acceleration_scaling);

  franka_motion::FrankaJointStateMonitor joint_state_monitor;
  auto joint_state_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    [&joint_state_monitor](const sensor_msgs::msg::JointState & message) {
      joint_state_monitor.update(message);
    });

  const auto stop_requested = []() {return !rclcpp::ok();};
  franka_motion::MoveItSequenceOperations operations;
  operations.capture_initial_joints = [&]() {
      return captureInitialJoints(
        move_group, joint_state_monitor, config, node->get_logger());
    };
  operations.move_to_pose = [&](const auto & pose, const auto & tag) {
      return franka_motion::planAndExecutePose(
        move_group, joint_state_monitor, node->get_logger(), pose, tag, 1,
        config.joint_state_max_age, stop_requested, config.current_state_timeout);
    };
  operations.move_to_joints = [&](const auto & joints, const auto & tag) {
      return franka_motion::planAndExecuteJoints(
        move_group, joint_state_monitor, node->get_logger(), joints, tag, 1,
        config.joint_state_max_age, stop_requested, config.current_state_timeout);
    };

  const bool succeeded = franka_motion::runMoveItSequence(
    operations, franka_motion::defaultMoveItSequenceTargets(), node->get_logger());
  (void)joint_state_subscription;
  return succeeded ? 0 : (rclcpp::ok() ? 1 : 130);
}

}  // namespace

/// Run the low-level diagnostic under the installed Panda profile.
int main(int argc, char ** argv)
{
  franka_motion::initializeWithRobotProfile(argc, argv);
  int exit_code = 1;
  try {
    exit_code = runDiagnostic();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("franka_moveit_sequence_node"),
      "MoveIt diagnostic завершился с исключением: %s", error.what());
  }
  const bool interrupted = !rclcpp::ok();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return interrupted ? 130 : exit_code;
}
