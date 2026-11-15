// Responsibility: Verifies catalog schema and pure primitive/mesh/compound conversion boundaries.

#include <array>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "shape_msgs/msg/mesh_triangle.hpp"

#include "manipulation/workflow/object_catalog.hpp"
#include "manipulation/workflow/object_geometry_conversion.hpp"
#include "transfer_domain_test_support.hpp"

namespace manipulation::workflow
{
namespace
{

using test_support::rigid_pose;
using test_support::stamped_pose;

class RecordingMeshLoader final : public MeshResourceLoader
{
public:
  shape_msgs::msg::Mesh load(
    const std::string_view resource_uri,
    const std::array<double, 3> & scale) const override
  {
    uris.emplace_back(resource_uri);
    scales.emplace_back(scale);

    shape_msgs::msg::Mesh mesh;
    geometry_msgs::msg::Point first;
    geometry_msgs::msg::Point second;
    geometry_msgs::msg::Point third;
    second.x = scale[0];
    third.y = scale[1];
    mesh.vertices = {first, second, third};
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices = {0U, 1U, 2U};
    mesh.triangles.emplace_back(triangle);
    return mesh;
  }

  mutable std::vector<std::string> uris;
  mutable std::vector<std::array<double, 3>> scales;
};

ObjectProfile primitive_profile()
{
  return ObjectProfile::create(
    "primitive_profile",
    ObjectGeometry::primitive(
      PrimitiveGeometry::create(
        PrimitiveGeometryType::kBox, {0.1, 0.2, 0.3}, rigid_pose(0.01, 0.02, 0.03))));
}

ObjectProfile mesh_profile()
{
  return ObjectProfile::create(
    "mesh_profile",
    ObjectGeometry::mesh(
      MeshGeometry::create(
        "package://demo/part.stl", {2.0, 3.0, 4.0}, rigid_pose(0.04, 0.05, 0.06))));
}

ObjectProfile compound_profile()
{
  std::vector<ObjectGeometryPart> parts;
  parts.emplace_back(
    PrimitiveGeometry::create(
      PrimitiveGeometryType::kSphere, {0.07}, rigid_pose(0.1, 0.0, 0.0)));
  parts.emplace_back(
    MeshGeometry::create(
      "package://demo/handle.stl", {1.0, 1.5, 2.0}, rigid_pose(0.0, 0.2, 0.0)));
  return ObjectProfile::create("compound_profile", ObjectGeometry::compound(std::move(parts)));
}

TEST(ObjectCatalog, SeparatesReusableProfilesFromPhysicalInstances)
{
  auto catalog = ObjectCatalog::create(
    {primitive_profile(), mesh_profile(), compound_profile()},
      {
        ObjectInstance::create("box_a", "primitive_profile"),
        ObjectInstance::create("box_b", "primitive_profile"),
        ObjectInstance::create("mesh_a", "mesh_profile"),
        ObjectInstance::create("tool_a", "compound_profile"),
    });

  EXPECT_EQ(catalog.profile_count(), 3U);
  EXPECT_EQ(catalog.instance_count(), 4U);
  EXPECT_EQ(catalog.profile_for("box_a").profile_id(), "primitive_profile");
  EXPECT_EQ(catalog.profile_for("box_b").profile_id(), "primitive_profile");
  EXPECT_EQ(catalog.require_profile("primitive_profile").geometry().kind(),
    ObjectGeometryKind::kPrimitive);
  EXPECT_EQ(catalog.require_profile("mesh_profile").geometry().kind(),
    ObjectGeometryKind::kMesh);
  EXPECT_EQ(catalog.require_profile("compound_profile").geometry().kind(),
    ObjectGeometryKind::kCompound);
  EXPECT_THROW(static_cast<void>(catalog.require_instance("missing")), std::out_of_range);
}

TEST(ObjectProfile, ValidatesAndPreservesLocalOrientationSymmetries)
{
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  geometry_msgs::msg::Quaternion quarter_turn;
  quarter_turn.z = 0.7071067811865476;
  quarter_turn.w = 0.7071067811865476;
  const auto base = primitive_profile();

  const auto profile = ObjectProfile::create(
    "symmetric_profile", base.geometry(), std::nullopt, {identity, quarter_turn});

  ASSERT_EQ(profile.orientation_symmetries().size(), 2U);
  EXPECT_DOUBLE_EQ(profile.orientation_symmetries().front().w, 1.0);
  EXPECT_DOUBLE_EQ(profile.orientation_symmetries().back().z, quarter_turn.z);
}

TEST(ObjectProfile, RejectsInvalidIncompleteOrDuplicateOrientationSymmetries)
{
  geometry_msgs::msg::Quaternion identity;
  identity.w = 1.0;
  geometry_msgs::msg::Quaternion quarter_turn;
  quarter_turn.z = 0.7071067811865476;
  quarter_turn.w = 0.7071067811865476;
  auto negative_quarter_turn = quarter_turn;
  negative_quarter_turn.z = -negative_quarter_turn.z;
  negative_quarter_turn.w = -negative_quarter_turn.w;
  auto invalid = identity;
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  const auto base = primitive_profile();

  EXPECT_THROW(
    static_cast<void>(ObjectProfile::create(
      "missing_identity", base.geometry(), std::nullopt, {quarter_turn})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(ObjectProfile::create(
      "invalid_symmetry", base.geometry(), std::nullopt, {identity, invalid})),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(ObjectProfile::create(
      "duplicate_symmetry", base.geometry(), std::nullopt,
      {identity, quarter_turn, negative_quarter_turn})),
    std::invalid_argument);
}

TEST(ObjectGeometryConversion, PreservesPrimitiveObjectAndPoseSemantics)
{
  const auto instance = ObjectInstance::create("box_a", "primitive_profile");
  const auto profile = primitive_profile();
  auto object_pose = stamped_pose("map", 1.25);
  RecordingMeshLoader loader;

  const auto result = convert_object_geometry(instance, profile, object_pose, loader);

  EXPECT_EQ(result.id, "box_a");
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_EQ(result.header.stamp.sec, object_pose.header.stamp.sec);
  EXPECT_DOUBLE_EQ(result.pose.position.x, 1.25);
  ASSERT_EQ(result.primitives.size(), 1U);
  ASSERT_EQ(result.primitive_poses.size(), 1U);
  EXPECT_EQ(result.primitives.front().type, shape_msgs::msg::SolidPrimitive::BOX);
  ASSERT_EQ(result.primitives.front().dimensions.size(), 3U);
  EXPECT_DOUBLE_EQ(result.primitives.front().dimensions.at(0), 0.1);
  EXPECT_DOUBLE_EQ(result.primitives.front().dimensions.at(1), 0.2);
  EXPECT_DOUBLE_EQ(result.primitives.front().dimensions.at(2), 0.3);
  EXPECT_DOUBLE_EQ(result.primitive_poses.front().position.y, 0.02);
  EXPECT_TRUE(result.meshes.empty());
  EXPECT_TRUE(loader.uris.empty());
}

TEST(ObjectGeometryConversion, DelegatesMeshResolutionAndPreservesMeshPose)
{
  const auto instance = ObjectInstance::create("mesh_a", "mesh_profile");
  const auto profile = mesh_profile();
  RecordingMeshLoader loader;

  const auto result = convert_object_geometry(instance, profile, stamped_pose(), loader);

  ASSERT_EQ(loader.uris.size(), 1U);
  EXPECT_EQ(loader.uris.front(), "package://demo/part.stl");
  EXPECT_EQ(loader.scales.front(), (std::array<double, 3>{2.0, 3.0, 4.0}));
  ASSERT_EQ(result.meshes.size(), 1U);
  ASSERT_EQ(result.mesh_poses.size(), 1U);
  EXPECT_DOUBLE_EQ(result.meshes.front().vertices.at(1).x, 2.0);
  EXPECT_DOUBLE_EQ(result.mesh_poses.front().position.z, 0.06);
  EXPECT_TRUE(result.primitives.empty());
}

TEST(ObjectGeometryConversion, CombinesPrimitiveAndMeshCompoundParts)
{
  const auto instance = ObjectInstance::create("tool_a", "compound_profile");
  const auto profile = compound_profile();
  RecordingMeshLoader loader;

  const auto result = convert_object_geometry(instance, profile, stamped_pose("odom"), loader);

  EXPECT_EQ(result.id, "tool_a");
  EXPECT_EQ(result.header.frame_id, "odom");
  ASSERT_EQ(result.primitives.size(), 1U);
  ASSERT_EQ(result.primitive_poses.size(), 1U);
  ASSERT_EQ(result.meshes.size(), 1U);
  ASSERT_EQ(result.mesh_poses.size(), 1U);
  EXPECT_EQ(result.primitives.front().type, shape_msgs::msg::SolidPrimitive::SPHERE);
  EXPECT_DOUBLE_EQ(result.primitive_poses.front().position.x, 0.1);
  EXPECT_DOUBLE_EQ(result.mesh_poses.front().position.y, 0.2);
  EXPECT_EQ(loader.uris, (std::vector<std::string>{"package://demo/handle.stl"}));
}

}  // namespace
}  // namespace manipulation::workflow
