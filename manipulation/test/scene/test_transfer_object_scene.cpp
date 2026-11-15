// Responsibility: Verifies complete staged geometry, resource sets, and pair-scoped diffs.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "manipulation/scene/coordinator_utils.hpp"
#include "manipulation/scene/transfer_object_scene.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace
{

moveit_msgs::msg::CollisionObject complete_object()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.header.stamp.sec = 12;
  object.id = "payload";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.06, 0.07};
  object.primitives.push_back(box);
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(box_pose);

  shape_msgs::msg::Mesh mesh;
  geometry_msgs::msg::Point vertex;
  vertex.x = 0.01;
  mesh.vertices.push_back(vertex);
  vertex.y = 0.01;
  mesh.vertices.push_back(vertex);
  vertex.z = 0.01;
  mesh.vertices.push_back(vertex);
  shape_msgs::msg::MeshTriangle triangle;
  triangle.vertex_indices = {0U, 1U, 2U};
  mesh.triangles.push_back(triangle);
  object.meshes.push_back(mesh);
  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.orientation.w = 1.0;
  object.mesh_poses.push_back(mesh_pose);
  return object;
}

}  // namespace

TEST(TransferObjectScene, BuildsDeterministicCompleteResourceSet)
{
  const auto resources = manipulation::scene::make_transfer_object_scene_resources(
    "payload", {"tool", "left_finger", "right_finger"});
  EXPECT_EQ(resources.world, "world:payload");
  EXPECT_EQ(resources.attached, "attached:payload");
  EXPECT_EQ(resources.allowed_collision_pairs.size(), 3U);
  EXPECT_TRUE(std::is_sorted(resources.all.begin(), resources.all.end()));
  EXPECT_THROW(
    manipulation::scene::make_transfer_object_scene_resources(
      "payload", {"tool", "tool"}),
    std::invalid_argument);
}

TEST(TransferObjectScene, PreservesCompleteStampedObjectInWorldDiff)
{
  const auto object = complete_object();
  std::string reason;
  ASSERT_TRUE(manipulation::scene::validate_staged_collision_object(
      object, "world", reason)) << reason;
  const auto diff = manipulation::scene::make_staged_world_add_diff(object, "world");
  ASSERT_EQ(diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(diff.world.collision_objects.front(), object);
  const auto validation = manipulation::scene::validate_scene_diff_resources(
    diff, {"world:payload"});
  EXPECT_TRUE(validation.valid) << validation.message;
}

TEST(TransferObjectScene, RejectsMissingStampFramePoseAndGeometry)
{
  std::string reason;
  auto object = complete_object();
  object.header.stamp = builtin_interfaces::msg::Time();
  EXPECT_FALSE(manipulation::scene::validate_staged_collision_object(
      object, "world", reason));
  object = complete_object();
  object.header.frame_id = "camera";
  EXPECT_FALSE(manipulation::scene::validate_staged_collision_object(
      object, "world", reason));
  object = complete_object();
  object.pose.orientation.x = 0.0;
  object.pose.orientation.y = 0.0;
  object.pose.orientation.z = 0.0;
  object.pose.orientation.w = 0.0;
  EXPECT_FALSE(manipulation::scene::validate_staged_collision_object(
      object, "world", reason));
  object = complete_object();
  object.primitives.clear();
  object.primitive_poses.clear();
  object.meshes.clear();
  object.mesh_poses.clear();
  EXPECT_FALSE(manipulation::scene::validate_staged_collision_object(
      object, "world", reason));
}

TEST(TransferObjectScene, BuildsExactPairAllowAndWorldRemoveDiffs)
{
  const auto allow = manipulation::scene::make_staged_contact_allow_diff(
    "payload", "tool");
  auto validation = manipulation::scene::validate_scene_diff_resources(
    allow, {"acm:payload|tool"});
  EXPECT_TRUE(validation.valid) << validation.message;
  ASSERT_EQ(allow.allowed_collision_matrix.entry_values.size(), 2U);
  EXPECT_TRUE(allow.allowed_collision_matrix.entry_values[0].enabled[1]);

  const auto remove = manipulation::scene::make_staged_world_remove_diff(
    "payload", "world");
  validation = manipulation::scene::validate_scene_diff_resources(
    remove, {"world:payload"});
  EXPECT_TRUE(validation.valid) << validation.message;
}
