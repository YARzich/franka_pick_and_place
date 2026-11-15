// Responsibility: Parse separated provider presets without task wait or transport policies.

#include "workflow_v2/provider_config_yaml.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

constexpr int kProviderSchemaVersion = 1;

const YAML::Node require_node(
  const YAML::Node & parent, const char * key, const YAML::NodeType::value type)
{
  const auto value = parent[key];
  if (!value || value.Type() != type) {
    throw std::invalid_argument(std::string{"provider field '"} + key + "' has wrong type");
  }
  return value;
}

void require_schema(const YAML::Node & root, const std::string & expected_provider)
{
  if (!root || !root.IsMap()) {
    throw std::invalid_argument("provider YAML root must be a mapping");
  }
  if (require_node(root, "schema_version", YAML::NodeType::Scalar).as<int>() !=
    kProviderSchemaVersion)
  {
    throw std::invalid_argument("unsupported provider schema_version");
  }
  if (require_node(root, "provider_id", YAML::NodeType::Scalar).as<std::string>() !=
    expected_provider)
  {
    throw std::invalid_argument("provider_id does not match loader");
  }
}

template<std::size_t Size>
std::array<double, Size> fixed_array(const YAML::Node & node, const char * field)
{
  if (!node || !node.IsSequence() || node.size() != Size) {
    throw std::invalid_argument(
            std::string{"provider field '"} + field + "' must have " +
            std::to_string(Size) + " elements");
  }
  std::array<double, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = node[index].as<double>();
  }
  return result;
}

Eigen::Vector3d unit_vector(const YAML::Node & node, const char * field)
{
  const auto values = fixed_array<3U>(node, field);
  Eigen::Vector3d result(values[0], values[1], values[2]);
  if (!result.allFinite() || std::abs(result.norm() - 1.0) > 1.0e-9) {
    throw std::invalid_argument(std::string{"provider field '"} + field + "' must be unit length");
  }
  return result;
}

geometry_msgs::msg::Pose rigid_pose(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("configured observation pose must be a mapping");
  }
  const auto position = fixed_array<3U>(
    require_node(node, "position_xyz", YAML::NodeType::Sequence), "position_xyz");
  const auto orientation = fixed_array<4U>(
    require_node(node, "orientation_xyzw", YAML::NodeType::Sequence), "orientation_xyzw");
  geometry_msgs::msg::Pose result;
  result.position.x = position[0];
  result.position.y = position[1];
  result.position.z = position[2];
  result.orientation.x = orientation[0];
  result.orientation.y = orientation[1];
  result.orientation.z = orientation[2];
  result.orientation.w = orientation[3];
  if (!manipulation::validation::is_valid_rigid_pose(result)) {
    throw std::invalid_argument("configured observation pose is not a finite rigid pose");
  }
  return result;
}

Eigen::Isometry3d rigid_transform(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("ik_frame transform must be a mapping");
  }
  const auto position = fixed_array<3U>(
    require_node(node, "offset_xyz", YAML::NodeType::Sequence), "offset_xyz");
  const auto orientation = fixed_array<4U>(
    require_node(node, "orientation_xyzw", YAML::NodeType::Sequence),
    "orientation_xyzw");
  Eigen::Quaterniond quaternion(
    orientation[3], orientation[0], orientation[1], orientation[2]);
  if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 1.0e-9) {
    throw std::invalid_argument("ik_frame orientation must be a unit quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(position[0], position[1], position[2]);
  result.linear() = quaternion.toRotationMatrix();
  if (!result.matrix().allFinite()) {
    throw std::invalid_argument("ik_frame transform must be finite");
  }
  return result;
}

std::string file_contents(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::invalid_argument("cannot open provider config '" + path.string() + "'");
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

template<typename Parse>
auto parse_yaml(const std::string_view yaml, Parse && parse)
{
  try {
    return parse(YAML::Load(std::string{yaml}));
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string{"invalid provider YAML: "} + error.what());
  }
}

}  // namespace

ConfiguredObservationPreset parse_configured_observation_yaml(const std::string_view yaml)
{
  return parse_yaml(yaml, [](const YAML::Node & root) {
             require_schema(root, "configured");
             const auto observations = require_node(root, "observations", YAML::NodeType::Map);
             ConfiguredObservationPreset result;
             for (const auto & entry : observations) {
               const auto object_id = entry.first.as<std::string>();
               const auto node = entry.second;
               ConfiguredObservation observation;
               observation.frame_id = require_node(
        node, "frame_id", YAML::NodeType::Scalar).as<std::string>();
               observation.pose = rigid_pose(node);
               if (node["confidence"]) {
                 observation.confidence = node["confidence"].as<double>();
               }
               const auto inserted = result.emplace(object_id, std::move(observation));
               if (!inserted.second) {
                 throw std::invalid_argument("configured provider contains duplicate object ID");
               }
             }
             if (result.empty()) {
               throw std::invalid_argument("configured provider requires at least one observation");
             }
             return result;
  });
}

ConfiguredObservationPreset load_configured_observation_yaml(
  const std::filesystem::path & path)
{
  return parse_configured_observation_yaml(file_contents(path));
}

DetectedObservationProviderConfig parse_detected_observation_yaml(const std::string_view yaml)
{
  return parse_yaml(yaml, [](const YAML::Node & root) {
             require_schema(root, "detected");
             DetectedObservationProviderConfig result;
             result.object_id = require_node(root, "object_id",
        YAML::NodeType::Scalar).as<std::string>();
             result.target_frame = require_node(
      root, "target_frame", YAML::NodeType::Scalar).as<std::string>();
             if (result.object_id.empty() ||
             !manipulation::validation::is_canonical_relative_frame_id(result.target_frame))
             {
               throw std::invalid_argument("detected provider object or target frame is invalid");
             }
             if (root["max_observation_age_sec"]) {
               result.max_observation_age_sec = require_node(
        root, "max_observation_age_sec", YAML::NodeType::Scalar).as<double>();
               if (!std::isfinite(result.max_observation_age_sec) ||
               result.max_observation_age_sec <= 0.0)
               {
                 throw std::invalid_argument(
                         "detected provider max_observation_age_sec must be finite and positive");
               }
             }
             if (root["minimum_position_z_m"]) {
               result.minimum_position_z = require_node(
        root, "minimum_position_z_m", YAML::NodeType::Scalar).as<double>();
               if (!std::isfinite(*result.minimum_position_z)) {
                 throw std::invalid_argument(
                         "detected provider minimum_position_z_m must be finite");
               }
             }
             return result;
  });
}

DetectedObservationProviderConfig load_detected_observation_yaml(
  const std::filesystem::path & path)
{
  return parse_detected_observation_yaml(file_contents(path));
}

MtcGraspProfiles parse_mtc_grasp_profiles_yaml(const std::string_view yaml)
{
  return parse_yaml(yaml, [](const YAML::Node & root) {
             require_schema(root, "mtc");
             const auto profiles = require_node(root, "grasp_profiles", YAML::NodeType::Map);
             MtcGraspProfiles result;
             for (const auto & entry : profiles) {
               const auto profile_id = entry.first.as<std::string>();
               const auto node = entry.second;
               MtcGraspProfileConfig config;
               config.arm_group_name = require_node(
        node, "arm_group", YAML::NodeType::Scalar).as<std::string>();
               config.eef_name = require_node(node, "eef",
          YAML::NodeType::Scalar).as<std::string>();
               config.ik_frame_link = require_node(
        node, "ik_frame_link", YAML::NodeType::Scalar).as<std::string>();
               config.pregrasp_pose = require_node(
        node, "pregrasp_named_pose", YAML::NodeType::Scalar).as<std::string>();
               config.grasp_pose = require_node(
        node, "grasp_named_pose", YAML::NodeType::Scalar).as<std::string>();
               config.ik_frame_transform = rigid_transform(
        require_node(node, "ik_frame", YAML::NodeType::Map));
               config.rotation_axis = unit_vector(
        require_node(node, "rotation_axis_xyz", YAML::NodeType::Sequence),
        "rotation_axis_xyz");
               config.approach_direction = unit_vector(
        require_node(node, "approach_direction_xyz", YAML::NodeType::Sequence),
        "approach_direction_xyz");
               config.angle_delta = require_node(
        node, "angle_delta_rad", YAML::NodeType::Scalar).as<double>();
               config.min_solution_distance = require_node(
        node, "min_solution_distance", YAML::NodeType::Scalar).as<double>();
               config.max_ik_solutions = require_node(
        node, "max_ik_solutions", YAML::NodeType::Scalar).as<std::size_t>();
               config.max_solutions = require_node(
        node, "max_solutions", YAML::NodeType::Scalar).as<std::size_t>();
               config.pregrasp_distance_m = require_node(
        node, "pregrasp_distance_m", YAML::NodeType::Scalar).as<double>();
               config.gripper_opening_m = require_node(
        node, "gripper_opening_m", YAML::NodeType::Scalar).as<double>();
               config.gripper_force_n = require_node(
        node, "gripper_force_n", YAML::NodeType::Scalar).as<double>();
               const auto inserted = result.emplace(profile_id, std::move(config));
               if (!inserted.second) {
                 throw std::invalid_argument("MTC provider contains duplicate grasp profile ID");
               }
             }
             if (result.empty()) {
               throw std::invalid_argument("MTC provider requires at least one grasp profile");
             }
             return result;
  });
}

MtcGraspProfiles load_mtc_grasp_profiles_yaml(const std::filesystem::path & path)
{
  return parse_mtc_grasp_profiles_yaml(file_contents(path));
}

}  // namespace franka_motion::workflow_v2
