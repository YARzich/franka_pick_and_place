// Responsibility: Verifies complete PlanningScene objects fail closed on malformed data.

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "geometry_msgs/msg/point32.hpp"
#include "manipulation/scene/collision_object_validation.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "shape_msgs/msg/mesh_triangle.hpp"
#include "shape_msgs/msg/plane.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace
{

moveit_msgs::msg::CollisionObject valid_box()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "workpiece";
  object.header.frame_id = "robot/base_link";
  object.pose.orientation.w = 1.0;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.1, 0.2, 0.3};
  object.primitives.push_back(primitive);

  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(primitive_pose);
  return object;
}

geometry_msgs::msg::Point32 polygon_point(const float x, const float y)
{
  geometry_msgs::msg::Point32 result;
  result.x = x;
  result.y = y;
  return result;
}

moveit_msgs::msg::CollisionObject valid_mesh_object()
{
  auto object = valid_box();
  object.primitives.clear();
  object.primitive_poses.clear();

  shape_msgs::msg::Mesh mesh;
  geometry_msgs::msg::Point vertex;
  mesh.vertices.push_back(vertex);
  vertex.x = 1.0;
  mesh.vertices.push_back(vertex);
  vertex.x = 0.0;
  vertex.y = 1.0;
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

bool valid(const moveit_msgs::msg::CollisionObject & object)
{
  return manipulation::scene::validate_explicit_collision_object(object).valid;
}

TEST(CollisionObjectValidation, AcceptsCompleteBoxAndNormalizedZeroStamp)
{
  const auto validation = manipulation::scene::validate_explicit_collision_object(valid_box());
  EXPECT_TRUE(validation.valid) << validation.message;
  EXPECT_TRUE(validation.message.empty());
}

TEST(CollisionObjectValidation, RejectsMalformedIdentityFrameAndStamp)
{
  auto object = valid_box();
  object.id = "bad/id/";
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.id = "ambiguous|resource";
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.header.frame_id = "robot//base_link";
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.header.stamp.sec = -1;
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.header.stamp.nanosec = 1000000000U;
  EXPECT_FALSE(valid(object));
}

TEST(CollisionObjectValidation, RejectsUnknownTypeWrongDimensionsAndMutationOperation)
{
  auto object = valid_box();
  object.primitives.front().type = 255U;
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.primitives.front().dimensions = {0.1};
  EXPECT_FALSE(valid(object));

  object = valid_box();
  object.operation = moveit_msgs::msg::CollisionObject::APPEND;
  EXPECT_FALSE(valid(object));
}

TEST(CollisionObjectValidation, RequiresFiniteCounterClockwisePrismPolygon)
{
  auto object = valid_box();
  auto & primitive = object.primitives.front();
  primitive.type = shape_msgs::msg::SolidPrimitive::PRISM;
  primitive.dimensions = {0.2};
  primitive.polygon.points = {
    polygon_point(0.0F, 0.0F), polygon_point(1.0F, 0.0F), polygon_point(0.0F, 1.0F)};
  EXPECT_TRUE(valid(object));

  std::reverse(primitive.polygon.points.begin(), primitive.polygon.points.end());
  EXPECT_FALSE(valid(object));
}

TEST(CollisionObjectValidation, RejectsInvalidOrDegenerateMeshTriangles)
{
  auto object = valid_mesh_object();
  EXPECT_TRUE(valid(object));

  object.meshes.front().triangles.front().vertex_indices = {0U, 0U, 2U};
  EXPECT_FALSE(valid(object));

  object = valid_mesh_object();
  object.meshes.front().vertices[2].x = 2.0;
  object.meshes.front().vertices[2].y = 0.0;
  EXPECT_FALSE(valid(object));

  object = valid_mesh_object();
  object.meshes.front().triangles.front().vertex_indices[2] = 3U;
  EXPECT_FALSE(valid(object));
}

TEST(CollisionObjectValidation, RejectsPlaneWithoutFiniteNonzeroNormal)
{
  auto object = valid_box();
  object.primitives.clear();
  object.primitive_poses.clear();

  shape_msgs::msg::Plane plane;
  plane.coef = {0.0, 0.0, 1.0, -0.4};
  object.planes.push_back(plane);
  geometry_msgs::msg::Pose plane_pose;
  plane_pose.orientation.w = 1.0;
  object.plane_poses.push_back(plane_pose);
  EXPECT_TRUE(valid(object));

  object.planes.front().coef = {0.0, 0.0, 0.0, 1.0};
  EXPECT_FALSE(valid(object));
  object.planes.front().coef[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid(object));
}

TEST(CollisionObjectValidation, RequiresPairedPosesAndUniqueValidSubframes)
{
  auto object = valid_box();
  object.primitive_poses.clear();
  EXPECT_FALSE(valid(object));

  object = valid_box();
  geometry_msgs::msg::Pose subframe_pose;
  subframe_pose.orientation.w = 1.0;
  object.subframe_names = {"grasp", "grasp"};
  object.subframe_poses = {subframe_pose, subframe_pose};
  EXPECT_FALSE(valid(object));

  object.subframe_names = {"grasp", "tool//center"};
  EXPECT_FALSE(valid(object));

  object.subframe_names = {"grasp", "tool/center"};
  EXPECT_TRUE(valid(object));
}

}  // namespace
