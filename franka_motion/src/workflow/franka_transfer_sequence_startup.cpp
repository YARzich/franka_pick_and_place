// Responsibility: Prepares the validated MoveIt snapshot used by the resident MTC planner.

#include "workflow/franka_transfer_sequence_startup.hpp"

#include <chrono>

namespace franka_motion
{

MoveItParameterSyncOptions make_franka_transfer_sequence_moveit_sync_options()
{
  MoveItParameterSyncOptions options;
  options.helper_node_name = "franka_transfer_sequence_parameter_helper";
  options.service_timeout = std::chrono::seconds{60};
  options.optional_timeout = std::chrono::seconds{2};
  options.service_unavailable_message =
    "move_group is unavailable; the Franka sequence server will not expose actions";
  options.required_prefixes = {
    "robot_description_kinematics",
    "robot_description_planning"};
  options.optional_prefixes = {
    "planning_pipelines",
    "ompl"};
  options.legacy_uninitialized_parameter_names =
    panda_urdf_position_limit_parameter_names();
  return options;
}

bool prepare_franka_transfer_sequence_moveit_snapshot(
  const rclcpp::Node::SharedPtr & node,
  std::function<bool()> stop_requested)
{
  if (!node) {
    return false;
  }
  const auto options = make_franka_transfer_sequence_moveit_sync_options();
  return copy_moveit_parameters_from_move_group(node, options, stop_requested) &&
         has_required_panda_acceleration_limits(node) &&
         publish_moveit_parameter_snapshot_manifest(node);
}

}  // namespace franka_motion
