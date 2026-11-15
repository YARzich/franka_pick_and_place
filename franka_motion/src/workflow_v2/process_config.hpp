// Responsibility: Defines the private, validated composition input for the Franka V2 workflow.

#ifndef WORKFLOW_V2__PROCESS_CONFIG_HPP_
#define WORKFLOW_V2__PROCESS_CONFIG_HPP_

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "manipulation/workflow/object_catalog.hpp"
#include "workflow_v2/provider_config_yaml.hpp"

namespace franka_motion::workflow_v2
{

inline constexpr std::string_view kConfiguredObservationProviderId{"configured"};
inline constexpr std::string_view kDetectedObservationProviderId{"detected"};
inline constexpr std::string_view kMtcGraspProviderId{"mtc"};

using PackageShareLookup =
  std::function<std::filesystem::path(std::string_view package_name)>;

/** Resolve one package resource for reading, rejecting traversal and missing files. */
[[nodiscard]] std::filesystem::path resolve_read_only_package_uri(
  std::string_view uri,
  const PackageShareLookup & package_share_lookup = {});

struct WorkflowProcessConfig
{
  std::string strategy_id;
  std::string observation_provider_id;
  std::string grasp_provider_id;
  std::filesystem::path object_catalog_file;
  std::filesystem::path observation_provider_config_file;
  std::filesystem::path grasp_provider_config_file;
  std::size_t max_steps{0U};
};

struct PickLiftPlaceProcessConfig
{
  double lift_clearance_m{0.0};
  double preplace_clearance_m{0.0};
  double retreat_clearance_m{0.0};
  std::chrono::steady_clock::duration release_settle{};
};

/** The only task-configurable command deadlines in the V2 process. */
struct PhysicalActionTimeouts
{
  std::chrono::steady_clock::duration arm_motion{};
  std::chrono::steady_clock::duration gripper_command{};
};

struct RobotProcessProfile
{
  std::string planning_frame;
  std::string planning_group;
  std::string end_effector_link;
  std::string attachment_link;
  std::vector<std::string> touch_links;
  double open_width_m{0.0};
  bool allow_grasp_stall{false};
  std::vector<std::string> home_joint_names;
  std::vector<double> home_joint_positions;
  std::string gripper_action;
};

struct MoveItBackendProfile
{
  std::string move_group_action;
  std::string execute_trajectory_action;
  std::string planning_pipeline;
  std::string planner_id;
  double velocity_scaling{0.0};
  double acceleration_scaling{0.0};
  double position_tolerance{0.0};
  double orientation_tolerance{0.0};
  double joint_tolerance{0.0};
};

struct DeploymentProfile
{
  std::string sequence_action;
  std::string sequence_pause_service;
  std::string detected_pose_topic;
  std::string health_topic;
  std::string scene_acquire_service;
  std::string scene_renew_service;
  std::string scene_validate_service;
  std::string scene_state_service;
  std::string scene_apply_service;
  std::string scene_release_service;
  std::string planning_scene_read_service;
  std::string physical_attachment_service;
};

using LoadedObservationProviderConfig =
  std::variant<ConfiguredObservationPreset, DetectedObservationProviderConfig>;

/** Complete startup snapshot; every referenced file has already been parsed. */
struct LoadedV2ProcessConfig
{
  WorkflowProcessConfig workflow;
  PickLiftPlaceProcessConfig pick_lift_place;
  PhysicalActionTimeouts physical_actions;
  RobotProcessProfile robot;
  MoveItBackendProfile moveit;
  DeploymentProfile deployment;
  manipulation::workflow::ObjectCatalog object_catalog;
  LoadedObservationProviderConfig observation_provider;
  MtcGraspProfiles grasp_profiles;
};

/** Read merged ROS parameters and reject invalid IDs, values, URIs, or profile files. */
[[nodiscard]] LoadedV2ProcessConfig load_v2_process_config(
  const rclcpp::Node::SharedPtr & node,
  const PackageShareLookup & package_share_lookup = {});

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__PROCESS_CONFIG_HPP_
