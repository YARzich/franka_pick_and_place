// Responsibility: Load one strict V2 process snapshot from separated ROS parameter profiles.

#include "workflow_v2/process_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/expand_topic_or_service_name.hpp"

#include "franka_motion/node_parameter_utils.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/workflow/strategy_registry.hpp"
#include "workflow_v2/object_catalog_yaml.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

constexpr std::string_view kDefaultCatalogUri{
  "package://franka_motion/config/tasks/object_catalog.yaml"};
constexpr std::string_view kDefaultObservationUri{
  "package://franka_motion/config/tasks/detected_observation.yaml"};
constexpr std::string_view kDefaultGraspUri{
  "package://franka_motion/config/tasks/mtc_grasp_profiles.yaml"};

template<typename T>
T parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  return franka_motion::declareOrGetParameter<T>(node, name, default_value);
}

bool valid_package_name(const std::string_view value)
{
  return !value.empty() &&
         std::all_of(value.cbegin(), value.cend(), [](const char character) {
             return (character >= 'a' && character <= 'z') ||
                    (character >= '0' && character <= '9') || character == '_';
         });
}

bool escapes_base(const std::filesystem::path & relative)
{
  return relative.empty() || relative.is_absolute() ||
         std::any_of(relative.begin(), relative.end(), [](const auto & component) {
             return component == "..";
         });
}

std::filesystem::path default_package_share(const std::string_view package_name)
{
  return ament_index_cpp::get_package_share_directory(std::string{package_name});
}

void require_nonempty(const std::string & value, const char * field)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string{field} + " must be non-empty");
  }
}

void require_frame(const std::string & value, const char * field)
{
  if (!manipulation::validation::is_canonical_relative_frame_id(value)) {
    throw std::invalid_argument(std::string{field} + " must be a canonical relative frame");
  }
}

void require_endpoint(const std::string & value, const char * field, const bool is_service)
{
  if (value.empty() || value.front() != '/') {
    throw std::invalid_argument(std::string{field} + " must be an absolute ROS name");
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      value, "franka_transfer_object_sequence_server", "/", is_service);
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            std::string{field} + " is not a valid ROS name: " + error.what());
  }
}

void require_positive(const double value, const char * field)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string{field} + " must be finite and positive");
  }
}

std::chrono::steady_clock::duration duration_from_seconds(
  const double value, const char * field, const bool allow_zero)
{
  if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0)) {
    throw std::invalid_argument(
            std::string{field} + (allow_zero ? " must be finite and non-negative" :
            " must be finite and positive"));
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{value});
}

WorkflowProcessConfig load_workflow(
  const rclcpp::Node::SharedPtr & node,
  const PackageShareLookup & lookup)
{
  WorkflowProcessConfig result;
  result.strategy_id = parameter<std::string>(
    node, "workflow.strategy_id", "pick_lift_place");
  result.observation_provider_id = parameter<std::string>(
    node, "workflow.observation_provider_id", std::string{kDetectedObservationProviderId});
  result.grasp_provider_id = parameter<std::string>(
    node, "workflow.grasp_provider_id", std::string{kMtcGraspProviderId});
  const auto catalog_uri = parameter<std::string>(
    node, "workflow.object_catalog_file", std::string{kDefaultCatalogUri});
  const auto observation_uri = parameter<std::string>(
    node, "workflow.observation_provider_config_file", std::string{kDefaultObservationUri});
  const auto grasp_uri = parameter<std::string>(
    node, "workflow.grasp_provider_config_file", std::string{kDefaultGraspUri});
  const auto max_steps = parameter<std::int64_t>(node, "workflow.max_steps", 32);

  if (result.strategy_id != manipulation::workflow::kPickLiftPlaceStrategyId) {
    throw std::invalid_argument("unknown workflow.strategy_id '" + result.strategy_id + "'");
  }
  if (result.observation_provider_id != kConfiguredObservationProviderId &&
    result.observation_provider_id != kDetectedObservationProviderId)
  {
    throw std::invalid_argument(
            "unknown workflow.observation_provider_id '" +
            result.observation_provider_id + "'");
  }
  if (result.grasp_provider_id != kMtcGraspProviderId) {
    throw std::invalid_argument(
            "unknown workflow.grasp_provider_id '" + result.grasp_provider_id + "'");
  }
  if (max_steps <= 0 || static_cast<std::uint64_t>(max_steps) >
    std::numeric_limits<std::size_t>::max())
  {
    throw std::invalid_argument("workflow.max_steps must fit the positive size range");
  }
  result.max_steps = static_cast<std::size_t>(max_steps);
  result.object_catalog_file = resolve_read_only_package_uri(catalog_uri, lookup);
  result.observation_provider_config_file =
    resolve_read_only_package_uri(observation_uri, lookup);
  result.grasp_provider_config_file = resolve_read_only_package_uri(grasp_uri, lookup);
  return result;
}

PickLiftPlaceProcessConfig load_pick_lift_place(const rclcpp::Node::SharedPtr & node)
{
  PickLiftPlaceProcessConfig result;
  result.lift_clearance_m = parameter<double>(
    node, "pick_lift_place.lift_clearance_m", 0.25);
  result.preplace_clearance_m = parameter<double>(
    node, "pick_lift_place.preplace_clearance_m", 0.12);
  result.retreat_clearance_m = parameter<double>(
    node, "pick_lift_place.retreat_clearance_m", 0.25);
  result.release_settle = duration_from_seconds(
    parameter<double>(node, "pick_lift_place.release_settle_sec", 0.3),
    "pick_lift_place.release_settle_sec", true);
  require_positive(result.lift_clearance_m, "pick_lift_place.lift_clearance_m");
  require_positive(result.preplace_clearance_m, "pick_lift_place.preplace_clearance_m");
  require_positive(result.retreat_clearance_m, "pick_lift_place.retreat_clearance_m");
  return result;
}

PhysicalActionTimeouts load_physical_actions(const rclcpp::Node::SharedPtr & node)
{
  PhysicalActionTimeouts result;
  result.arm_motion = duration_from_seconds(
    parameter<double>(node, "physical_actions.arm_motion_timeout_sec", 30.0),
    "physical_actions.arm_motion_timeout_sec", false);
  result.gripper_command = duration_from_seconds(
    parameter<double>(node, "physical_actions.gripper_command_timeout_sec", 3.0),
    "physical_actions.gripper_command_timeout_sec", false);
  return result;
}

RobotProcessProfile load_robot(const rclcpp::Node::SharedPtr & node)
{
  RobotProcessProfile result;
  result.planning_frame = parameter<std::string>(node, "robot.planning_frame", "panda_link0");
  result.planning_group = parameter<std::string>(node, "robot.planning_group", "panda_arm");
  result.end_effector_link = parameter<std::string>(
    node, "robot.end_effector_link", "panda_grasp_frame");
  result.attachment_link = parameter<std::string>(
    node, "robot.attachment_link", "panda_hand");
  result.touch_links = parameter<std::vector<std::string>>(
    node, "robot.touch_links", {"panda_hand", "panda_leftfinger", "panda_rightfinger"});
  result.open_width_m = parameter<double>(node, "robot.open_width_m", 0.08);
  result.allow_grasp_stall = parameter<bool>(node, "robot.allow_grasp_stall", true);
  result.home_joint_names = parameter<std::vector<std::string>>(
    node, "robot.home_joint_names",
    {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
      "panda_joint5", "panda_joint6", "panda_joint7"});
  result.home_joint_positions = parameter<std::vector<double>>(
    node, "robot.home_joint_positions", {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});
  result.gripper_action = parameter<std::string>(
    node, "robot.gripper_action", "/panda_gripper/gripper_cmd");

  require_frame(result.planning_frame, "robot.planning_frame");
  require_nonempty(result.planning_group, "robot.planning_group");
  require_frame(result.end_effector_link, "robot.end_effector_link");
  require_frame(result.attachment_link, "robot.attachment_link");
  if (result.touch_links.empty() || result.home_joint_names.empty() ||
    result.home_joint_names.size() != result.home_joint_positions.size())
  {
    throw std::invalid_argument("robot link/home arrays are empty or have mismatched sizes");
  }
  for (const auto & link : result.touch_links) {
    require_frame(link, "robot.touch_links");
  }
  for (const auto & joint : result.home_joint_names) {
    require_nonempty(joint, "robot.home_joint_names");
  }
  if (std::set<std::string>(result.touch_links.cbegin(), result.touch_links.cend()).size() !=
    result.touch_links.size() ||
    std::set<std::string>(
      result.home_joint_names.cbegin(), result.home_joint_names.cend()).size() !=
    result.home_joint_names.size() ||
    !std::all_of(
      result.home_joint_positions.cbegin(), result.home_joint_positions.cend(),
      [](const double value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("robot link/home arrays contain duplicates or invalid values");
  }
  if (!std::isfinite(result.open_width_m) || result.open_width_m < 0.0) {
    throw std::invalid_argument("robot.open_width_m must be finite and non-negative");
  }
  require_endpoint(result.gripper_action, "robot.gripper_action", false);
  return result;
}

MoveItBackendProfile load_moveit(const rclcpp::Node::SharedPtr & node)
{
  MoveItBackendProfile result;
  result.move_group_action = parameter<std::string>(
    node, "moveit.move_group_action", "/move_action");
  result.execute_trajectory_action = parameter<std::string>(
    node, "moveit.execute_trajectory_action", "/execute_trajectory");
  result.planning_pipeline = parameter<std::string>(node, "moveit.planning_pipeline", "");
  result.planner_id = parameter<std::string>(node, "moveit.planner_id", "");
  result.velocity_scaling = parameter<double>(node, "moveit.velocity_scaling", 0.2);
  result.acceleration_scaling = parameter<double>(node, "moveit.acceleration_scaling", 0.2);
  result.position_tolerance = parameter<double>(node, "moveit.position_tolerance", 0.001);
  result.orientation_tolerance = parameter<double>(
    node, "moveit.orientation_tolerance", 0.02);
  result.joint_tolerance = parameter<double>(node, "moveit.joint_tolerance", 0.01);

  require_endpoint(result.move_group_action, "moveit.move_group_action", false);
  require_endpoint(
    result.execute_trajectory_action, "moveit.execute_trajectory_action", false);
  const auto valid_scaling = [](const double value) {
      return std::isfinite(value) && value > 0.0 && value <= 1.0;
    };
  if (!valid_scaling(result.velocity_scaling) ||
    !valid_scaling(result.acceleration_scaling))
  {
    throw std::invalid_argument("MoveIt scaling values must be in (0, 1]");
  }
  require_positive(result.position_tolerance, "moveit.position_tolerance");
  require_positive(result.orientation_tolerance, "moveit.orientation_tolerance");
  require_positive(result.joint_tolerance, "moveit.joint_tolerance");
  return result;
}

DeploymentProfile load_deployment(const rclcpp::Node::SharedPtr & node)
{
  DeploymentProfile result;
  result.sequence_action = parameter<std::string>(
    node, "endpoints.sequence_action", "/transfer_object_sequence_v2");
  result.sequence_pause_service = parameter<std::string>(
    node, "endpoints.sequence_pause_service", "/set_transfer_object_sequence_v2_paused");
  result.detected_pose_topic = parameter<std::string>(
    node, "endpoints.detected_pose_topic", "/table_stereo/cube_pose");
  result.health_topic = parameter<std::string>(
    node, "endpoints.health_topic", "/system_health_monitor/health");
  result.scene_acquire_service = parameter<std::string>(
    node, "endpoints.scene_acquire_service", "/planning_scene_coordinator/acquire_lease");
  result.scene_renew_service = parameter<std::string>(
    node, "endpoints.scene_renew_service", "/planning_scene_coordinator/renew_lease");
  result.scene_validate_service = parameter<std::string>(
    node, "endpoints.scene_validate_service", "/planning_scene_coordinator/validate_lease");
  result.scene_state_service = parameter<std::string>(
    node, "endpoints.scene_state_service", "/planning_scene_coordinator/get_state");
  result.scene_apply_service = parameter<std::string>(
    node, "endpoints.scene_apply_service", "/planning_scene_coordinator/apply_scene_diff");
  result.scene_release_service = parameter<std::string>(
    node, "endpoints.scene_release_service", "/planning_scene_coordinator/release_lease");
  result.planning_scene_read_service = parameter<std::string>(
    node, "endpoints.planning_scene_read_service", "/get_planning_scene");
  result.physical_attachment_service = parameter<std::string>(
    node, "endpoints.physical_attachment_service", "");

  require_endpoint(result.sequence_action, "endpoints.sequence_action", false);
  require_endpoint(
    result.sequence_pause_service, "endpoints.sequence_pause_service", true);
  require_endpoint(result.detected_pose_topic, "endpoints.detected_pose_topic", false);
  require_endpoint(result.health_topic, "endpoints.health_topic", false);
  require_endpoint(result.scene_acquire_service, "endpoints.scene_acquire_service", true);
  require_endpoint(result.scene_renew_service, "endpoints.scene_renew_service", true);
  require_endpoint(result.scene_validate_service, "endpoints.scene_validate_service", true);
  require_endpoint(result.scene_state_service, "endpoints.scene_state_service", true);
  require_endpoint(result.scene_apply_service, "endpoints.scene_apply_service", true);
  require_endpoint(result.scene_release_service, "endpoints.scene_release_service", true);
  require_endpoint(
    result.planning_scene_read_service, "endpoints.planning_scene_read_service", true);
  if (!result.physical_attachment_service.empty()) {
    require_endpoint(
      result.physical_attachment_service,
      "endpoints.physical_attachment_service", true);
  }
  return result;
}

LoadedObservationProviderConfig load_observation_provider(
  const WorkflowProcessConfig & workflow)
{
  if (workflow.observation_provider_id == kConfiguredObservationProviderId) {
    return load_configured_observation_yaml(workflow.observation_provider_config_file);
  }
  return load_detected_observation_yaml(workflow.observation_provider_config_file);
}

void validate_provider_bindings(
  const RobotProcessProfile & robot,
  const manipulation::workflow::ObjectCatalog & catalog,
  const LoadedObservationProviderConfig & observation,
  const MtcGraspProfiles & grasps)
{
  for (const auto & [profile_id, profile] : grasps) {
    (void)profile_id;
    if (profile.arm_group_name != robot.planning_group) {
      throw std::invalid_argument("MTC grasp profile uses a different robot planning group");
    }
  }
  const auto validate_object = [&catalog, &grasps](const std::string & object_id) {
      const auto * instance = catalog.find_instance(object_id);
      if (!instance) {
        throw std::invalid_argument("observation provider references unknown object '" +
                object_id + "'");
      }
      const auto & profile = catalog.require_profile(instance->profile_id());
      if (!profile.grasp_profile_id() ||
        grasps.find(*profile.grasp_profile_id()) == grasps.cend())
      {
        throw std::invalid_argument("object profile has no matching MTC grasp profile");
      }
    };
  if (const auto * configured = std::get_if<ConfiguredObservationPreset>(&observation)) {
    for (const auto & [object_id, configured_observation] : *configured) {
      validate_object(object_id);
      if (configured_observation.frame_id != robot.planning_frame) {
        throw std::invalid_argument("configured observation is not in robot.planning_frame");
      }
    }
  } else {
    const auto & detected = std::get<DetectedObservationProviderConfig>(observation);
    validate_object(detected.object_id);
    if (detected.target_frame != robot.planning_frame) {
      throw std::invalid_argument(
            "detected provider target_frame differs from robot.planning_frame");
    }
  }
}

}  // namespace

std::filesystem::path resolve_read_only_package_uri(
  const std::string_view uri,
  const PackageShareLookup & package_share_lookup)
{
  constexpr std::string_view scheme{"package://"};
  if (uri.substr(0U, scheme.size()) != scheme) {
    throw std::invalid_argument("resource path must use package:// URI syntax");
  }
  const auto body = uri.substr(scheme.size());
  const auto separator = body.find('/');
  if (separator == std::string_view::npos || separator == 0U ||
    separator + 1U >= body.size())
  {
    throw std::invalid_argument("package URI must contain a package and relative file path");
  }
  const auto package_name = body.substr(0U, separator);
  const std::filesystem::path relative{std::string{body.substr(separator + 1U)}};
  if (!valid_package_name(package_name) || escapes_base(relative)) {
    throw std::invalid_argument("package URI contains an invalid package or traversal path");
  }

  try {
    const auto lookup = package_share_lookup ? package_share_lookup : default_package_share;
    const auto share = std::filesystem::weakly_canonical(lookup(package_name));
    const auto resolved = std::filesystem::weakly_canonical(share / relative);
    const auto within_share = resolved.lexically_relative(share);
    if (!std::filesystem::is_directory(share) || escapes_base(within_share) ||
      !std::filesystem::is_regular_file(resolved))
    {
      throw std::invalid_argument("package URI does not resolve to a readable package file");
    }
    std::ifstream stream(resolved);
    if (!stream) {
      throw std::invalid_argument("package resource cannot be opened for reading");
    }
    return resolved;
  } catch (const std::invalid_argument &) {
    throw;
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            "cannot resolve package URI '" + std::string{uri} + "': " + error.what());
  }
}

LoadedV2ProcessConfig load_v2_process_config(
  const rclcpp::Node::SharedPtr & node,
  const PackageShareLookup & package_share_lookup)
{
  if (!node) {
    throw std::invalid_argument("V2 process config requires a ROS node");
  }
  auto workflow = load_workflow(node, package_share_lookup);
  auto pick_lift_place = load_pick_lift_place(node);
  auto physical_actions = load_physical_actions(node);
  auto robot = load_robot(node);
  auto moveit = load_moveit(node);
  auto deployment = load_deployment(node);
  auto catalog = load_object_catalog_yaml(workflow.object_catalog_file);
  auto observation = load_observation_provider(workflow);
  auto grasp_profiles = load_mtc_grasp_profiles_yaml(workflow.grasp_provider_config_file);
  validate_provider_bindings(robot, catalog, observation, grasp_profiles);
  return LoadedV2ProcessConfig{
    std::move(workflow), std::move(pick_lift_place), std::move(physical_actions),
    std::move(robot), std::move(moveit), std::move(deployment), std::move(catalog),
    std::move(observation), std::move(grasp_profiles)};
}

}  // namespace franka_motion::workflow_v2
