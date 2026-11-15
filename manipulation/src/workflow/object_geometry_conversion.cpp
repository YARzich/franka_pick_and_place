// Responsibility: Performs deterministic catalog-to-CollisionObject conversion
//   without resource I/O.

#include "manipulation/workflow/object_geometry_conversion.hpp"

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "shape_msgs/msg/solid_primitive.hpp"

#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::workflow
{
namespace
{

std::uint8_t to_message_type(const PrimitiveGeometryType type)
{
  switch (type) {
    case PrimitiveGeometryType::kBox:
      return shape_msgs::msg::SolidPrimitive::BOX;
    case PrimitiveGeometryType::kSphere:
      return shape_msgs::msg::SolidPrimitive::SPHERE;
    case PrimitiveGeometryType::kCylinder:
      return shape_msgs::msg::SolidPrimitive::CYLINDER;
    case PrimitiveGeometryType::kCone:
      return shape_msgs::msg::SolidPrimitive::CONE;
  }
  throw std::invalid_argument("object profile contains an unknown primitive type");
}

void append_primitive(
  const PrimitiveGeometry & source,
  moveit_msgs::msg::CollisionObject & destination)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = to_message_type(source.type());
  primitive.dimensions.assign(source.dimensions().cbegin(), source.dimensions().cend());
  destination.primitives.emplace_back(std::move(primitive));
  destination.primitive_poses.emplace_back(source.local_pose());
}

bool is_structurally_valid_mesh(const shape_msgs::msg::Mesh & mesh)
{
  if (mesh.vertices.empty() || mesh.triangles.empty()) {
    return false;
  }
  for (const auto & vertex : mesh.vertices) {
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z)) {
      return false;
    }
  }
  for (const auto & triangle : mesh.triangles) {
    for (const auto index : triangle.vertex_indices) {
      if (index >= mesh.vertices.size()) {
        return false;
      }
    }
  }
  return true;
}

void append_mesh(
  const MeshGeometry & source,
  const MeshResourceLoader & loader,
  moveit_msgs::msg::CollisionObject & destination)
{
  auto mesh = loader.load(source.resource_uri(), source.scale());
  if (!is_structurally_valid_mesh(mesh)) {
    throw std::invalid_argument(
            "mesh loader returned empty, non-finite, or index-invalid geometry for '" +
            source.resource_uri() + "'");
  }
  destination.meshes.emplace_back(std::move(mesh));
  destination.mesh_poses.emplace_back(source.local_pose());
}

}  // namespace

moveit_msgs::msg::CollisionObject convert_object_geometry(
  const ObjectInstance & instance,
  const ObjectProfile & profile,
  const geometry_msgs::msg::PoseStamped & object_pose,
  const MeshResourceLoader & mesh_loader)
{
  if (instance.profile_id() != profile.profile_id()) {
    throw std::invalid_argument("object geometry conversion received a mismatched profile");
  }
  if (!manipulation::validation::is_valid_pose_stamped(object_pose)) {
    throw std::invalid_argument("object geometry conversion requires a valid stamped object pose");
  }

  moveit_msgs::msg::CollisionObject result;
  result.header = object_pose.header;
  result.pose = object_pose.pose;
  result.id = instance.object_id();
  result.operation = moveit_msgs::msg::CollisionObject::ADD;

  for (const auto & part : profile.geometry().parts()) {
    std::visit(
      [&result, &mesh_loader](const auto & value) {
        using Part = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<Part, PrimitiveGeometry>) {
          append_primitive(value, result);
        } else {
          append_mesh(value, mesh_loader, result);
        }
      },
      part);
  }
  return result;
}

}  // namespace manipulation::workflow
