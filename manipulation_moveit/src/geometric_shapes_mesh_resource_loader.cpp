// Responsibility: Loads and scales catalog meshes through geometric_shapes resources.

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "manipulation_moveit/ros_v2_scene_adapter.hpp"

#include "Eigen/Core"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shapes.h"
#include "shape_msgs/msg/mesh_triangle.hpp"

namespace manipulation::moveit
{

shape_msgs::msg::Mesh GeometricShapesMeshResourceLoader::load(
  const std::string_view resource_uri,
  const std::array<double, 3> & scale) const
{
  if (resource_uri.empty() ||
    !std::all_of(scale.cbegin(), scale.cend(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    }))
  {
    throw std::invalid_argument("mesh resource URI and XYZ scale must be valid");
  }

  std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
      std::string{resource_uri}, Eigen::Vector3d{scale[0], scale[1], scale[2]}));
  if (!mesh || mesh->vertex_count == 0U || mesh->triangle_count == 0U ||
    mesh->vertices == nullptr || mesh->triangles == nullptr)
  {
    throw std::runtime_error(
            "geometric_shapes could not load mesh resource '" +
            std::string{resource_uri} + "'");
  }

  shape_msgs::msg::Mesh result;
  result.vertices.reserve(mesh->vertex_count);
  for (unsigned int index = 0U; index < mesh->vertex_count; ++index) {
    geometry_msgs::msg::Point vertex;
    vertex.x = mesh->vertices[index * 3U];
    vertex.y = mesh->vertices[index * 3U + 1U];
    vertex.z = mesh->vertices[index * 3U + 2U];
    result.vertices.push_back(vertex);
  }
  result.triangles.reserve(mesh->triangle_count);
  for (unsigned int index = 0U; index < mesh->triangle_count; ++index) {
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices[0] = mesh->triangles[index * 3U];
    triangle.vertex_indices[1] = mesh->triangles[index * 3U + 1U];
    triangle.vertex_indices[2] = mesh->triangles[index * 3U + 2U];
    result.triangles.push_back(triangle);
  }
  return result;
}

}  // namespace manipulation::moveit
