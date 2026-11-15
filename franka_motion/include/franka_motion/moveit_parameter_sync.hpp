// Responsibility: Declares atomic synchronization of MoveIt parameters into standalone planning
//   nodes.

#ifndef FRANKA_MOTION__MOVEIT_PARAMETER_SYNC_HPP_
#define FRANKA_MOTION__MOVEIT_PARAMETER_SYNC_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

#include "franka_motion/moveit_parameter_snapshot_contract.hpp"

namespace franka_motion
{

/**
 * Selects the immutable MoveIt parameters copied into one consumer node.
 *
 * The helper node is created only for the duration of the copy. Discovery and
 * reads are bounded by steady-clock durations; ROS or simulation time cannot
 * extend startup. Required entries fail the complete copy, while an optional
 * name or prefix is applied atomically only when its whole snapshot is valid.
 */
struct MoveItParameterSyncOptions
{
  std::string helper_node_name;
  std::string move_group_node_name{"move_group"};
  std::chrono::seconds service_timeout{15};
  std::chrono::seconds optional_timeout{1};
  std::string service_unavailable_message{
    "move_group недоступен, сначала запусти MoveIt launch"};
  std::vector<std::string> required_names{
    "robot_description",
    "robot_description_semantic"};
  std::vector<std::string> optional_names{
    "use_sim_time",
    "planning_pipelines",
    "default_planning_pipeline",
    "moveit_manage_controllers",
    "moveit_controller_manager",
    "publish_planning_scene",
    "publish_geometry_updates",
    "publish_state_updates",
    "publish_robot_description",
    "publish_robot_description_semantic"};
  std::vector<std::string> required_prefixes;
  std::vector<std::string> optional_prefixes{"robot_description_kinematics"};
  // A legacy source may list these URDF-owned parameters without assigning a value.
  // Initialized values are still copied; a manifest always remains authoritative.
  std::vector<std::string> legacy_uninitialized_parameter_names;
  std::string prefixed_names_manifest{kMoveItParameterSnapshotManifest};
};

/// Return whether the local node owns an initialized, non-empty string parameter.
bool has_non_empty_string_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name);

/// Return whether the local node has any parameter at or below `prefix`.
bool has_parameter_prefix(
  const rclcpp::Node::SharedPtr & node,
  const std::string & prefix);

/** URDF-owned Panda position-limit names that MoveIt may declare without values. */
std::vector<std::string> panda_urdf_position_limit_parameter_names();

/**
 * Copy a bounded snapshot from one MoveIt parameter source.
 *
 * Required data and manifest transport errors fail closed. Each prefix is
 * staged before one atomic local parameter update; rejected optional snapshots
 * are skipped without exposing partially updated values. A source without a
 * manifest is assumed to keep its startup parameters immutable while read.
 * The function performs no executor spin on `node`; its private helper node owns
 * the temporary executor used for parameter-service responses.
 *
 * \return false when service discovery, required data, the manifest or a local
 * atomic update fails; true after all required snapshots were installed.
 */
bool copy_moveit_parameters_from_move_group(
  const rclcpp::Node::SharedPtr & node,
  const MoveItParameterSyncOptions & options);

/**
 * Copy a snapshot with cooperative cancellation during service and response waits.
 *
 * This overload keeps the original options type ABI-stable. A true or throwing
 * callback stops the complete operation and returns false, including optional
 * copy phases that would otherwise be fail-soft.
 */
bool copy_moveit_parameters_from_move_group(
  const rclcpp::Node::SharedPtr & node,
  const MoveItParameterSyncOptions & options,
  const std::function<bool()> & stop_requested);

/**
 * Publish the immutable prefix snapshot available from this consumer node.
 * Existing values are replaced atomically; an empty snapshot fails closed.
 */
bool publish_moveit_parameter_snapshot_manifest(
  const rclcpp::Node::SharedPtr & node,
  const std::vector<std::string> & prefixes = {
    "robot_description_kinematics",
    "robot_description_planning",
    "planning_pipelines",
    "ompl"
  });

/**
 * Fail closed unless all Panda arm and finger joints have enabled, finite,
 * positive acceleration limits in the local MoveIt parameter snapshot.
 */
bool has_required_panda_acceleration_limits(const rclcpp::Node::SharedPtr & node);

}  // namespace franka_motion

#endif  // FRANKA_MOTION__MOVEIT_PARAMETER_SYNC_HPP_
