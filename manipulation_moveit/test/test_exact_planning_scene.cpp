// Responsibility: Prove exact scene construction never inherits hidden MoveGroup world state.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

#include "exact_planning_scene.hpp"

namespace manipulation::moveit::detail
{
namespace
{

moveit_msgs::msg::CollisionObject box(const std::string & id, const double x)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "panda_link0";
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.1, 0.1, 0.1};
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.orientation.w = 1.0;
  object.primitives.push_back(std::move(primitive));
  object.primitive_poses.push_back(std::move(pose));
  return object;
}

TEST(ExactPlanningSceneTest, FullSnapshotDropsBackendOnlyStaleObject)
{
  const auto model = ::moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);

  planning_scene::PlanningScene backend_scene(model);
  ASSERT_TRUE(backend_scene.processCollisionObjectMsg(box("backend-only-stale", 0.2)));
  ASSERT_TRUE(backend_scene.getWorld()->hasObject("backend-only-stale"));

  planning_scene::PlanningScene authoritative_scene(model);
  ASSERT_TRUE(authoritative_scene.processCollisionObjectMsg(box("authoritative", 0.6)));
  moveit_msgs::msg::PlanningScene snapshot;
  authoritative_scene.getPlanningSceneMsg(snapshot);
  ASSERT_FALSE(snapshot.is_diff);
  ASSERT_FALSE(snapshot.robot_state.is_diff);

  std::string error;
  const auto exact_scene = make_exact_planning_scene(model, snapshot, error);

  ASSERT_NE(exact_scene, nullptr) << error;
  EXPECT_TRUE(exact_scene->getWorld()->hasObject("authoritative"));
  EXPECT_FALSE(exact_scene->getWorld()->hasObject("backend-only-stale"));
  EXPECT_TRUE(backend_scene.getWorld()->hasObject("backend-only-stale"));
}

TEST(ExactPlanningSceneTest, DiffInputIsRejected)
{
  const auto model = ::moveit::core::loadTestingRobotModel("panda");
  ASSERT_NE(model, nullptr);
  moveit_msgs::msg::PlanningScene snapshot;
  snapshot.is_diff = true;
  snapshot.robot_state.is_diff = false;

  std::string error;
  EXPECT_EQ(make_exact_planning_scene(model, snapshot, error), nullptr);
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace manipulation::moveit::detail
