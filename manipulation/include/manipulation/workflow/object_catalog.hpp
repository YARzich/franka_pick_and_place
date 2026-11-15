// Responsibility: Defines validated object instances, reusable geometry profiles,
//   and catalog lookup.

#ifndef MANIPULATION__WORKFLOW__OBJECT_CATALOG_HPP_
#define MANIPULATION__WORKFLOW__OBJECT_CATALOG_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace manipulation::workflow
{

/// Primitive shapes supported by the object-profile schema.
enum class PrimitiveGeometryType : std::uint8_t
{
  kBox,
  kSphere,
  kCylinder,
  kCone,
};

/** One validated primitive and its pose in the object profile frame. */
class PrimitiveGeometry final
{
public:
  /** Validate the shape-specific dimensions and local rigid pose. */
  [[nodiscard]] static PrimitiveGeometry create(
    PrimitiveGeometryType type,
    std::vector<double> dimensions,
    geometry_msgs::msg::Pose local_pose);

  [[nodiscard]] PrimitiveGeometryType type() const noexcept;
  [[nodiscard]] const std::vector<double> & dimensions() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::Pose & local_pose() const noexcept;

private:
  PrimitiveGeometry(
    PrimitiveGeometryType type,
    std::vector<double> dimensions,
    geometry_msgs::msg::Pose local_pose);

  PrimitiveGeometryType type_;
  std::vector<double> dimensions_;
  geometry_msgs::msg::Pose local_pose_;
};

/** One mesh resource and its scale and pose in the object profile frame. */
class MeshGeometry final
{
public:
  /** Validate a non-empty resource URI, positive XYZ scale, and local rigid pose. */
  [[nodiscard]] static MeshGeometry create(
    std::string resource_uri,
    std::array<double, 3> scale,
    geometry_msgs::msg::Pose local_pose);

  [[nodiscard]] const std::string & resource_uri() const noexcept;
  [[nodiscard]] const std::array<double, 3> & scale() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::Pose & local_pose() const noexcept;

private:
  MeshGeometry(
    std::string resource_uri,
    std::array<double, 3> scale,
    geometry_msgs::msg::Pose local_pose);

  std::string resource_uri_;
  std::array<double, 3> scale_;
  geometry_msgs::msg::Pose local_pose_;
};

using ObjectGeometryPart = std::variant<PrimitiveGeometry, MeshGeometry>;

/// Top-level schema alternative retained after profile validation.
enum class ObjectGeometryKind : std::uint8_t
{
  kPrimitive,
  kMesh,
  kCompound,
};

/** Immutable primitive, mesh, or compound profile geometry. */
class ObjectGeometry final
{
public:
  [[nodiscard]] static ObjectGeometry primitive(PrimitiveGeometry primitive);
  [[nodiscard]] static ObjectGeometry mesh(MeshGeometry mesh);

  /** A compound contains at least two validated primitive or mesh parts. */
  [[nodiscard]] static ObjectGeometry compound(std::vector<ObjectGeometryPart> parts);

  [[nodiscard]] ObjectGeometryKind kind() const noexcept;
  [[nodiscard]] const std::vector<ObjectGeometryPart> & parts() const noexcept;

private:
  ObjectGeometry(ObjectGeometryKind kind, std::vector<ObjectGeometryPart> parts);

  ObjectGeometryKind kind_;
  std::vector<ObjectGeometryPart> parts_;
};

/** Reusable type-level object information, independent from a physical instance. */
class ObjectProfile final
{
public:
  [[nodiscard]] static ObjectProfile create(
    std::string profile_id,
    ObjectGeometry geometry,
    std::optional<std::string> grasp_profile_id = std::nullopt,
    std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries = {});

  [[nodiscard]] const std::string & profile_id() const noexcept;
  [[nodiscard]] const ObjectGeometry & geometry() const noexcept;
  [[nodiscard]] const std::optional<std::string> & grasp_profile_id() const noexcept;

  /**
   * Proper local-frame rotations that leave the object's physical state equivalent.
   *
   * An empty collection means that no symmetry is declared. A non-empty collection
   * contains the identity exactly once along with every supported equivalent rotation.
   */
  [[nodiscard]] const std::vector<geometry_msgs::msg::Quaternion> &
  orientation_symmetries() const noexcept;

private:
  ObjectProfile(
    std::string profile_id,
    ObjectGeometry geometry,
    std::optional<std::string> grasp_profile_id,
    std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries);

  std::string profile_id_;
  ObjectGeometry geometry_;
  std::optional<std::string> grasp_profile_id_;
  std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries_;
};

/** Identity of one physical object and its unambiguous profile binding. */
class ObjectInstance final
{
public:
  [[nodiscard]] static ObjectInstance create(
    std::string object_id,
    std::string profile_id);

  [[nodiscard]] const std::string & object_id() const noexcept;
  [[nodiscard]] const std::string & profile_id() const noexcept;

private:
  ObjectInstance(std::string object_id, std::string profile_id);

  std::string object_id_;
  std::string profile_id_;
};

/** Immutable lookup owning validated instance-to-profile bindings. */
class ObjectCatalog final
{
public:
  /**
   * Convert complete profile and instance schemas into one catalog.
   *
   * IDs must be unique, every instance must reference a present profile, and
   * both collections must be non-empty. Invalid schemas throw
   * `std::invalid_argument` before the catalog becomes observable.
   */
  [[nodiscard]] static ObjectCatalog create(
    std::vector<ObjectProfile> profiles,
    std::vector<ObjectInstance> instances);

  [[nodiscard]] const ObjectInstance * find_instance(std::string_view object_id) const noexcept;
  [[nodiscard]] const ObjectProfile * find_profile(std::string_view profile_id) const noexcept;

  /** Throw `std::out_of_range` when the requested instance is absent. */
  [[nodiscard]] const ObjectInstance & require_instance(std::string_view object_id) const;

  /** Throw `std::out_of_range` when the requested profile is absent. */
  [[nodiscard]] const ObjectProfile & require_profile(std::string_view profile_id) const;

  /** Resolve one instance to its validated profile binding. */
  [[nodiscard]] const ObjectProfile & profile_for(std::string_view object_id) const;

  [[nodiscard]] std::size_t profile_count() const noexcept;
  [[nodiscard]] std::size_t instance_count() const noexcept;

private:
  ObjectCatalog(
    std::map<std::string, ObjectProfile> profiles,
    std::map<std::string, ObjectInstance> instances);

  std::map<std::string, ObjectProfile> profiles_;
  std::map<std::string, ObjectInstance> instances_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__OBJECT_CATALOG_HPP_
