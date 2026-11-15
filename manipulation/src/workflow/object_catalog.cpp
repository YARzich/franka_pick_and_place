// Responsibility: Validates object geometry schemas and builds deterministic catalog lookup.

#include "manipulation/workflow/object_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::workflow
{
namespace
{

constexpr std::size_t kMaximumIdentifierLength = 256U;

bool is_visible_token(const std::string_view value)
{
  return !value.empty() && value.size() <= kMaximumIdentifierLength &&
         value.find('|') == std::string_view::npos &&
         std::all_of(value.cbegin(), value.cend(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return byte >= 0x21U && byte <= 0x7eU;
         });
}

void require_identifier(const std::string_view value, const char * field)
{
  if (!is_visible_token(value)) {
    throw std::invalid_argument(std::string{field} + " must be a bounded visible token");
  }
}

std::size_t expected_primitive_dimension_count(const PrimitiveGeometryType type)
{
  switch (type) {
    case PrimitiveGeometryType::kBox:
      return 3U;
    case PrimitiveGeometryType::kSphere:
      return 1U;
    case PrimitiveGeometryType::kCylinder:
    case PrimitiveGeometryType::kCone:
      return 2U;
  }
  throw std::invalid_argument("primitive geometry type is unknown");
}

void validate_local_pose(const geometry_msgs::msg::Pose & pose)
{
  if (!manipulation::validation::is_valid_rigid_pose(pose)) {
    throw std::invalid_argument("object geometry local pose must be finite with a unit quaternion");
  }
}

double quaternion_dot(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w;
}

void validate_orientation_symmetries(
  const std::vector<geometry_msgs::msg::Quaternion> & symmetries)
{
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  bool has_identity = false;
  for (std::size_t index = 0U; index < symmetries.size(); ++index) {
    const auto & symmetry = symmetries[index];
    if (!manipulation::validation::is_unit_quaternion(symmetry)) {
      throw std::invalid_argument(
              "object orientation symmetry must be finite with a unit quaternion");
    }
    has_identity = has_identity ||
      std::abs(quaternion_dot(symmetry, identity)) >=
      1.0 - manipulation::validation::kRigidPoseTolerance;
    const auto duplicate = std::any_of(
      symmetries.cbegin(), symmetries.cbegin() + static_cast<std::ptrdiff_t>(index),
      [&symmetry](const auto & prior) {
        return std::abs(quaternion_dot(symmetry, prior)) >=
               1.0 - manipulation::validation::kRigidPoseTolerance;
      });
    if (duplicate) {
      throw std::invalid_argument(
              "object orientation symmetries contain duplicate rotations");
    }
  }
  if (!symmetries.empty() && !has_identity) {
    throw std::invalid_argument(
            "object orientation symmetries must contain the identity rotation");
  }
}

}  // namespace

PrimitiveGeometry PrimitiveGeometry::create(
  const PrimitiveGeometryType type,
  std::vector<double> dimensions,
  geometry_msgs::msg::Pose local_pose)
{
  const auto expected_count = expected_primitive_dimension_count(type);
  if (dimensions.size() != expected_count ||
    !std::all_of(dimensions.cbegin(), dimensions.cend(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    }))
  {
    throw std::invalid_argument(
            "primitive geometry dimensions have an invalid count or non-positive value");
  }
  validate_local_pose(local_pose);
  return PrimitiveGeometry(type, std::move(dimensions), std::move(local_pose));
}

PrimitiveGeometry::PrimitiveGeometry(
  const PrimitiveGeometryType type,
  std::vector<double> dimensions,
  geometry_msgs::msg::Pose local_pose)
: type_(type), dimensions_(std::move(dimensions)), local_pose_(std::move(local_pose))
{
}

PrimitiveGeometryType PrimitiveGeometry::type() const noexcept
{
  return type_;
}

const std::vector<double> & PrimitiveGeometry::dimensions() const noexcept
{
  return dimensions_;
}

const geometry_msgs::msg::Pose & PrimitiveGeometry::local_pose() const noexcept
{
  return local_pose_;
}

MeshGeometry MeshGeometry::create(
  std::string resource_uri,
  const std::array<double, 3> scale,
  geometry_msgs::msg::Pose local_pose)
{
  require_identifier(resource_uri, "mesh resource URI");
  if (!std::all_of(scale.cbegin(), scale.cend(), [](const double value) {
      return std::isfinite(value) && value > 0.0;
    }))
  {
    throw std::invalid_argument("mesh scale must contain finite positive XYZ values");
  }
  validate_local_pose(local_pose);
  return MeshGeometry(std::move(resource_uri), scale, std::move(local_pose));
}

MeshGeometry::MeshGeometry(
  std::string resource_uri,
  const std::array<double, 3> scale,
  geometry_msgs::msg::Pose local_pose)
: resource_uri_(std::move(resource_uri)), scale_(scale), local_pose_(std::move(local_pose))
{
}

const std::string & MeshGeometry::resource_uri() const noexcept
{
  return resource_uri_;
}

const std::array<double, 3> & MeshGeometry::scale() const noexcept
{
  return scale_;
}

const geometry_msgs::msg::Pose & MeshGeometry::local_pose() const noexcept
{
  return local_pose_;
}

ObjectGeometry ObjectGeometry::primitive(PrimitiveGeometry primitive)
{
  std::vector<ObjectGeometryPart> parts;
  parts.emplace_back(std::move(primitive));
  return ObjectGeometry(ObjectGeometryKind::kPrimitive, std::move(parts));
}

ObjectGeometry ObjectGeometry::mesh(MeshGeometry mesh)
{
  std::vector<ObjectGeometryPart> parts;
  parts.emplace_back(std::move(mesh));
  return ObjectGeometry(ObjectGeometryKind::kMesh, std::move(parts));
}

ObjectGeometry ObjectGeometry::compound(std::vector<ObjectGeometryPart> parts)
{
  if (parts.size() < 2U) {
    throw std::invalid_argument("compound object geometry requires at least two parts");
  }
  return ObjectGeometry(ObjectGeometryKind::kCompound, std::move(parts));
}

ObjectGeometry::ObjectGeometry(
  const ObjectGeometryKind kind,
  std::vector<ObjectGeometryPart> parts)
: kind_(kind), parts_(std::move(parts))
{
}

ObjectGeometryKind ObjectGeometry::kind() const noexcept
{
  return kind_;
}

const std::vector<ObjectGeometryPart> & ObjectGeometry::parts() const noexcept
{
  return parts_;
}

ObjectProfile ObjectProfile::create(
  std::string profile_id,
  ObjectGeometry geometry,
  std::optional<std::string> grasp_profile_id,
  std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries)
{
  require_identifier(profile_id, "object profile ID");
  if (grasp_profile_id) {
    require_identifier(*grasp_profile_id, "grasp profile ID");
  }
  validate_orientation_symmetries(orientation_symmetries);
  return ObjectProfile(
    std::move(profile_id), std::move(geometry), std::move(grasp_profile_id),
    std::move(orientation_symmetries));
}

ObjectProfile::ObjectProfile(
  std::string profile_id,
  ObjectGeometry geometry,
  std::optional<std::string> grasp_profile_id,
  std::vector<geometry_msgs::msg::Quaternion> orientation_symmetries)
: profile_id_(std::move(profile_id)), geometry_(std::move(geometry)),
  grasp_profile_id_(std::move(grasp_profile_id)),
  orientation_symmetries_(std::move(orientation_symmetries))
{
}

const std::string & ObjectProfile::profile_id() const noexcept
{
  return profile_id_;
}

const ObjectGeometry & ObjectProfile::geometry() const noexcept
{
  return geometry_;
}

const std::optional<std::string> & ObjectProfile::grasp_profile_id() const noexcept
{
  return grasp_profile_id_;
}

const std::vector<geometry_msgs::msg::Quaternion> &
ObjectProfile::orientation_symmetries() const noexcept
{
  return orientation_symmetries_;
}

ObjectInstance ObjectInstance::create(std::string object_id, std::string profile_id)
{
  require_identifier(object_id, "object instance ID");
  require_identifier(profile_id, "object profile ID");
  return ObjectInstance(std::move(object_id), std::move(profile_id));
}

ObjectInstance::ObjectInstance(std::string object_id, std::string profile_id)
: object_id_(std::move(object_id)), profile_id_(std::move(profile_id))
{
}

const std::string & ObjectInstance::object_id() const noexcept
{
  return object_id_;
}

const std::string & ObjectInstance::profile_id() const noexcept
{
  return profile_id_;
}

ObjectCatalog ObjectCatalog::create(
  std::vector<ObjectProfile> profiles,
  std::vector<ObjectInstance> instances)
{
  if (profiles.empty() || instances.empty()) {
    throw std::invalid_argument("object catalog requires profiles and physical instances");
  }

  std::map<std::string, ObjectProfile> profile_lookup;
  for (auto & profile : profiles) {
    const auto id = profile.profile_id();
    const auto inserted = profile_lookup.emplace(id, std::move(profile));
    if (!inserted.second) {
      throw std::invalid_argument("object catalog contains duplicate profile ID '" + id + "'");
    }
  }

  std::map<std::string, ObjectInstance> instance_lookup;
  for (auto & instance : instances) {
    const auto object_id = instance.object_id();
    const auto profile_id = instance.profile_id();
    if (profile_lookup.find(profile_id) == profile_lookup.end()) {
      throw std::invalid_argument(
              "object instance '" + object_id + "' references unknown profile '" +
              profile_id + "'");
    }
    const auto inserted = instance_lookup.emplace(object_id, std::move(instance));
    if (!inserted.second) {
      throw std::invalid_argument(
              "object catalog contains duplicate instance ID '" + object_id + "'");
    }
  }
  return ObjectCatalog(std::move(profile_lookup), std::move(instance_lookup));
}

ObjectCatalog::ObjectCatalog(
  std::map<std::string, ObjectProfile> profiles,
  std::map<std::string, ObjectInstance> instances)
: profiles_(std::move(profiles)), instances_(std::move(instances))
{
}

const ObjectInstance * ObjectCatalog::find_instance(const std::string_view object_id) const noexcept
{
  const auto found = instances_.find(std::string{object_id});
  return found == instances_.cend() ? nullptr : &found->second;
}

const ObjectProfile * ObjectCatalog::find_profile(const std::string_view profile_id) const noexcept
{
  const auto found = profiles_.find(std::string{profile_id});
  return found == profiles_.cend() ? nullptr : &found->second;
}

const ObjectInstance & ObjectCatalog::require_instance(const std::string_view object_id) const
{
  const auto * instance = find_instance(object_id);
  if (!instance) {
    throw std::out_of_range("unknown object instance '" + std::string{object_id} + "'");
  }
  return *instance;
}

const ObjectProfile & ObjectCatalog::require_profile(const std::string_view profile_id) const
{
  const auto * profile = find_profile(profile_id);
  if (!profile) {
    throw std::out_of_range("unknown object profile '" + std::string{profile_id} + "'");
  }
  return *profile;
}

const ObjectProfile & ObjectCatalog::profile_for(const std::string_view object_id) const
{
  return require_profile(require_instance(object_id).profile_id());
}

std::size_t ObjectCatalog::profile_count() const noexcept
{
  return profiles_.size();
}

std::size_t ObjectCatalog::instance_count() const noexcept
{
  return instances_.size();
}

}  // namespace manipulation::workflow
