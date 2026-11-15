// Responsibility: Validates complete CollisionObject geometry, poses, identity, and metadata.

#include "manipulation/scene/collision_object_validation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace manipulation::scene
{
namespace
{

CollisionObjectValidationResult invalid(const std::string & message)
{
  return {false, message};
}

bool valid_relative_component(const std::string_view value)
{
  if (value.empty() || value.front() == '/' || value.back() == '/') {
    return false;
  }

  bool previous_was_separator = false;
  return std::none_of(
    value.cbegin(), value.cend(), [&previous_was_separator](const unsigned char character) {
      const bool separator = character == '/';
      const bool invalid_character = std::isspace(character) != 0 ||
      std::iscntrl(character) != 0 || (separator && previous_was_separator);
      previous_was_separator = separator;
      return invalid_character;
    });
}

bool valid_scene_component(const std::string_view value)
{
  return valid_relative_component(value) && value.find('|') == std::string_view::npos;
}

template<typename PoseRange>
bool valid_poses(const PoseRange & poses)
{
  return std::all_of(
    poses.cbegin(), poses.cend(), manipulation::validation::is_valid_rigid_pose);
}

bool valid_positive_dimensions(
  const shape_msgs::msg::SolidPrimitive & primitive,
  const std::size_t expected_size)
{
  return primitive.dimensions.size() == expected_size &&
         std::all_of(
    primitive.dimensions.cbegin(), primitive.dimensions.cend(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    });
}

bool valid_prism_polygon(const geometry_msgs::msg::Polygon & polygon)
{
  if (polygon.points.size() < 3U) {
    return false;
  }

  // shape_msgs defines a prism by a finite counter-clockwise polygon in XY.
  double twice_signed_area = 0.0;
  for (std::size_t index = 0; index < polygon.points.size(); ++index) {
    const auto & current = polygon.points[index];
    const auto & next = polygon.points[(index + 1U) % polygon.points.size()];
    if (!std::isfinite(current.x) || !std::isfinite(current.y) ||
      !std::isfinite(current.z))
    {
      return false;
    }
    twice_signed_area +=
      static_cast<double>(current.x) * static_cast<double>(next.y) -
      static_cast<double>(next.x) * static_cast<double>(current.y);
  }
  return std::isfinite(twice_signed_area) &&
         twice_signed_area > std::numeric_limits<double>::epsilon();
}

bool valid_primitive(const shape_msgs::msg::SolidPrimitive & primitive)
{
  using Primitive = shape_msgs::msg::SolidPrimitive;
  switch (primitive.type) {
    case Primitive::BOX:
      return primitive.polygon.points.empty() && valid_positive_dimensions(primitive, 3U);
    case Primitive::SPHERE:
      return primitive.polygon.points.empty() && valid_positive_dimensions(primitive, 1U);
    case Primitive::CYLINDER:
    case Primitive::CONE:
      return primitive.polygon.points.empty() && valid_positive_dimensions(primitive, 2U);
    case Primitive::PRISM:
      return valid_positive_dimensions(primitive, 1U) &&
             valid_prism_polygon(primitive.polygon);
    default:
      return false;
  }
}

bool valid_mesh(const shape_msgs::msg::Mesh & mesh)
{
  if (mesh.vertices.empty() || mesh.triangles.empty()) {
    return false;
  }
  const auto finite_vertex = [](const auto & vertex) {
      return std::isfinite(vertex.x) && std::isfinite(vertex.y) &&
             std::isfinite(vertex.z);
    };
  if (!std::all_of(mesh.vertices.cbegin(), mesh.vertices.cend(), finite_vertex)) {
    return false;
  }

  // Reject repeated indices and zero-area triangles before they reach FCL.
  return std::all_of(mesh.triangles.cbegin(), mesh.triangles.cend(),
           [&mesh](const auto & triangle) {
             const auto first = triangle.vertex_indices[0];
             const auto second = triangle.vertex_indices[1];
             const auto third = triangle.vertex_indices[2];
             if (first >= mesh.vertices.size() || second >= mesh.vertices.size() ||
             third >= mesh.vertices.size() || first == second || first == third ||
             second == third)
             {
               return false;
             }

             const auto & a = mesh.vertices[first];
             const auto & b = mesh.vertices[second];
             const auto & c = mesh.vertices[third];
             const double ab_x = b.x - a.x;
             const double ab_y = b.y - a.y;
             const double ab_z = b.z - a.z;
             const double ac_x = c.x - a.x;
             const double ac_y = c.y - a.y;
             const double ac_z = c.z - a.z;
             const double cross_x = ab_y * ac_z - ab_z * ac_y;
             const double cross_y = ab_z * ac_x - ab_x * ac_z;
             const double cross_z = ab_x * ac_y - ab_y * ac_x;
             const double area_squared =
             cross_x * cross_x + cross_y * cross_y + cross_z * cross_z;
             return std::isfinite(area_squared) &&
                    area_squared > std::numeric_limits<double>::epsilon();
  });
}

bool valid_plane(const shape_msgs::msg::Plane & plane)
{
  const bool finite =
    std::all_of(plane.coef.cbegin(), plane.coef.cend(), [](const double value) {
        return std::isfinite(value);
    });
  return finite &&
         std::hypot(plane.coef[0], plane.coef[1], plane.coef[2]) >
         std::numeric_limits<double>::epsilon();
}

bool valid_subframe_names(const std::vector<std::string> & names)
{
  std::unordered_set<std::string> unique_names;
  return std::all_of(names.cbegin(), names.cend(), [&unique_names](const auto & name) {
             return valid_scene_component(name) && unique_names.insert(name).second;
  });
}

}  // namespace

CollisionObjectValidationResult validate_explicit_collision_object(
  const moveit_msgs::msg::CollisionObject & object)
{
  if (!valid_scene_component(object.id)) {
    return invalid("object ID must be a canonical relative component");
  }
  if (!manipulation::validation::is_canonical_relative_frame_id(
      object.header.frame_id))
  {
    return invalid("object frame_id must be a canonical relative frame");
  }
  if (!manipulation::validation::is_normalized_stamp(object.header.stamp)) {
    return invalid("object stamp must be non-negative and normalized");
  }
  if (object.operation != moveit_msgs::msg::CollisionObject::ADD) {
    return invalid("explicit object operation must be ADD");
  }
  if (!manipulation::validation::is_valid_rigid_pose(object.pose)) {
    return invalid("object pose must be finite with a unit quaternion");
  }

  const bool geometry_present =
    !object.primitives.empty() || !object.meshes.empty() || !object.planes.empty();
  if (!geometry_present || object.primitives.size() != object.primitive_poses.size() ||
    object.meshes.size() != object.mesh_poses.size() ||
    object.planes.size() != object.plane_poses.size())
  {
    return invalid("object must contain explicit geometry and one pose per shape");
  }
  if (!std::all_of(object.primitives.cbegin(), object.primitives.cend(), valid_primitive)) {
    return invalid("object contains an invalid solid primitive");
  }
  if (!std::all_of(object.meshes.cbegin(), object.meshes.cend(), valid_mesh)) {
    return invalid("object contains an invalid or degenerate mesh");
  }
  if (!std::all_of(object.planes.cbegin(), object.planes.cend(), valid_plane)) {
    return invalid("object contains a plane without a finite non-zero normal");
  }
  if (!valid_poses(object.primitive_poses) || !valid_poses(object.mesh_poses) ||
    !valid_poses(object.plane_poses))
  {
    return invalid("object contains an invalid shape pose");
  }
  if (object.subframe_names.size() != object.subframe_poses.size() ||
    !valid_poses(object.subframe_poses) || !valid_subframe_names(object.subframe_names))
  {
    return invalid("object subframes must have unique names and valid paired poses");
  }
  return {true, {}};
}

}  // namespace manipulation::scene
