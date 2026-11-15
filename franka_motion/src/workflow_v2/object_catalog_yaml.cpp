// Responsibility: Converts the versioned Panda catalog YAML into validated generic values.

#include "workflow_v2/object_catalog_yaml.hpp"

#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace franka_motion::workflow_v2
{
namespace
{

using manipulation::workflow::MeshGeometry;
using manipulation::workflow::ObjectCatalog;
using manipulation::workflow::ObjectGeometry;
using manipulation::workflow::ObjectGeometryPart;
using manipulation::workflow::ObjectInstance;
using manipulation::workflow::ObjectProfile;
using manipulation::workflow::PrimitiveGeometry;
using manipulation::workflow::PrimitiveGeometryType;

constexpr int kCatalogSchemaVersion = 2;

const YAML::Node require_node(
  const YAML::Node & parent, const char * key, const YAML::NodeType::value type)
{
  const auto value = parent[key];
  if (!value || value.Type() != type) {
    throw std::invalid_argument(std::string{"catalog field '"} + key + "' has wrong type");
  }
  return value;
}

template<std::size_t Size>
std::array<double, Size> fixed_array(const YAML::Node & node, const char * field)
{
  if (!node || !node.IsSequence() || node.size() != Size) {
    throw std::invalid_argument(
            std::string{"catalog field '"} + field + "' must have " +
            std::to_string(Size) + " elements");
  }
  std::array<double, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = node[index].as<double>();
  }
  return result;
}

geometry_msgs::msg::Pose local_pose(const YAML::Node & node)
{
  geometry_msgs::msg::Pose result;
  result.orientation.w = 1.0;
  if (!node) {
    return result;
  }
  if (!node.IsMap()) {
    throw std::invalid_argument("catalog local_pose must be a mapping");
  }
  if (const auto position = node["position_xyz"]) {
    const auto values = fixed_array<3U>(position, "position_xyz");
    result.position.x = values[0];
    result.position.y = values[1];
    result.position.z = values[2];
  }
  if (const auto orientation = node["orientation_xyzw"]) {
    const auto values = fixed_array<4U>(orientation, "orientation_xyzw");
    result.orientation.x = values[0];
    result.orientation.y = values[1];
    result.orientation.z = values[2];
    result.orientation.w = values[3];
  }
  return result;
}

std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries(
  const YAML::Node & node)
{
  std::vector<geometry_msgs::msg::Quaternion> result;
  if (!node) {
    return result;
  }
  if (!node.IsSequence()) {
    throw std::invalid_argument(
            "catalog field 'orientation_symmetries_xyzw' must be a sequence");
  }
  result.reserve(node.size());
  for (const auto & entry : node) {
    const auto values = fixed_array<4U>(entry, "orientation_symmetries_xyzw entry");
    geometry_msgs::msg::Quaternion symmetry;
    symmetry.x = values[0];
    symmetry.y = values[1];
    symmetry.z = values[2];
    symmetry.w = values[3];
    result.emplace_back(std::move(symmetry));
  }
  return result;
}

PrimitiveGeometryType primitive_type(const std::string & value)
{
  if (value == "box") {
    return PrimitiveGeometryType::kBox;
  }
  if (value == "sphere") {
    return PrimitiveGeometryType::kSphere;
  }
  if (value == "cylinder") {
    return PrimitiveGeometryType::kCylinder;
  }
  if (value == "cone") {
    return PrimitiveGeometryType::kCone;
  }
  throw std::invalid_argument("catalog primitive has unsupported type '" + value + "'");
}

PrimitiveGeometry parse_primitive(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("catalog primitive geometry must be a mapping");
  }
  const auto type = require_node(node, "primitive_type", YAML::NodeType::Scalar).as<std::string>();
  const auto dimensions_node =
    require_node(node, "dimensions_m", YAML::NodeType::Sequence);
  return PrimitiveGeometry::create(
    primitive_type(type), dimensions_node.as<std::vector<double>>(),
        local_pose(node["local_pose"]));
}

MeshGeometry parse_mesh(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("catalog mesh geometry must be a mapping");
  }
  const auto resource = require_node(
    node, "resource_uri", YAML::NodeType::Scalar).as<std::string>();
  std::array<double, 3U> scale{1.0, 1.0, 1.0};
  if (node["scale_xyz"]) {
    scale = fixed_array<3U>(node["scale_xyz"], "scale_xyz");
  }
  return MeshGeometry::create(std::move(resource), scale, local_pose(node["local_pose"]));
}

ObjectGeometryPart parse_geometry_part(const YAML::Node & node)
{
  const auto kind = require_node(node, "kind", YAML::NodeType::Scalar).as<std::string>();
  if (kind == "primitive") {
    return parse_primitive(node);
  }
  if (kind == "mesh") {
    return parse_mesh(node);
  }
  throw std::invalid_argument("compound geometry part has unsupported kind '" + kind + "'");
}

ObjectGeometry parse_geometry(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("catalog geometry must be a mapping");
  }
  const auto kind = require_node(node, "kind", YAML::NodeType::Scalar).as<std::string>();
  if (kind == "primitive") {
    return ObjectGeometry::primitive(parse_primitive(node));
  }
  if (kind == "mesh") {
    return ObjectGeometry::mesh(parse_mesh(node));
  }
  if (kind == "compound") {
    const auto parts = require_node(node, "parts", YAML::NodeType::Sequence);
    std::vector<ObjectGeometryPart> converted;
    converted.reserve(parts.size());
    for (const auto & part : parts) {
      converted.emplace_back(parse_geometry_part(part));
    }
    return ObjectGeometry::compound(std::move(converted));
  }
  throw std::invalid_argument("catalog geometry has unsupported kind '" + kind + "'");
}

ObjectCatalog parse_root(const YAML::Node & root)
{
  if (!root || !root.IsMap()) {
    throw std::invalid_argument("object catalog root must be a mapping");
  }
  const int version = require_node(
    root, "schema_version", YAML::NodeType::Scalar).as<int>();
  if (version != kCatalogSchemaVersion) {
    throw std::invalid_argument("unsupported object catalog schema_version");
  }

  const auto profiles_node = require_node(root, "profiles", YAML::NodeType::Map);
  std::vector<ObjectProfile> profiles;
  profiles.reserve(profiles_node.size());
  for (const auto & entry : profiles_node) {
    const auto profile_id = entry.first.as<std::string>();
    const auto profile_node = entry.second;
    if (!profile_node.IsMap()) {
      throw std::invalid_argument("catalog object profile must be a mapping");
    }
    std::optional<std::string> grasp_profile_id;
    if (profile_node["grasp_profile_id"]) {
      grasp_profile_id = profile_node["grasp_profile_id"].as<std::string>();
    }
    profiles.emplace_back(ObjectProfile::create(
      profile_id, parse_geometry(profile_node["geometry"]), std::move(grasp_profile_id),
      orientation_symmetries(profile_node["orientation_symmetries_xyzw"])));
  }

  const auto objects_node = require_node(root, "objects", YAML::NodeType::Map);
  std::vector<ObjectInstance> objects;
  objects.reserve(objects_node.size());
  for (const auto & entry : objects_node) {
    const auto object_id = entry.first.as<std::string>();
    const auto object_node = entry.second;
    const auto profile_id = require_node(
      object_node, "profile_id", YAML::NodeType::Scalar).as<std::string>();
    objects.emplace_back(ObjectInstance::create(object_id, profile_id));
  }
  return ObjectCatalog::create(std::move(profiles), std::move(objects));
}

}  // namespace

ObjectCatalog parse_object_catalog_yaml(const std::string_view yaml)
{
  try {
    return parse_root(YAML::Load(std::string{yaml}));
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string{"invalid object catalog YAML: "} + error.what());
  }
}

ObjectCatalog load_object_catalog_yaml(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::invalid_argument("cannot open object catalog '" + path.string() + "'");
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return parse_object_catalog_yaml(contents.str());
}

}  // namespace franka_motion::workflow_v2
