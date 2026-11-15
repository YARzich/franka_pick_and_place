// Responsibility: Converts catalog geometry to explicit MoveIt objects through
//   an injected mesh port.

#ifndef MANIPULATION__WORKFLOW__OBJECT_GEOMETRY_CONVERSION_HPP_
#define MANIPULATION__WORKFLOW__OBJECT_GEOMETRY_CONVERSION_HPP_

#include <array>
#include <string_view>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"

#include "manipulation/workflow/object_catalog.hpp"

namespace manipulation::workflow
{

/** External boundary responsible for resolving and scaling one mesh resource. */
class MeshResourceLoader
{
public:
  virtual ~MeshResourceLoader() = default;

  /**
   * Return mesh vertices already transformed by the requested XYZ scale.
   *
   * Implementations own resource lookup and I/O. The domain converter never
   * opens files, resolves package paths, or depends on a concrete mesh library.
   */
  [[nodiscard]] virtual shape_msgs::msg::Mesh load(
    std::string_view resource_uri,
    const std::array<double, 3> & scale) const = 0;
};

/**
 * Convert one catalog-bound profile into a complete world-object ADD value.
 *
 * `object_pose.header` becomes the CollisionObject header, `object_pose.pose`
 * becomes its top-level object pose, and profile part poses remain relative to
 * that object pose. The instance ID is the MoveIt object ID.
 */
[[nodiscard]] moveit_msgs::msg::CollisionObject convert_object_geometry(
  const ObjectInstance & instance,
  const ObjectProfile & profile,
  const geometry_msgs::msg::PoseStamped & object_pose,
  const MeshResourceLoader & mesh_loader);

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__OBJECT_GEOMETRY_CONVERSION_HPP_
