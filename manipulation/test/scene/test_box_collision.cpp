// Responsibility: Verifies scene package box validation and exact collision-object construction.

#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "manipulation/scene/box_collision.hpp"

namespace
{

manipulation::scene::BoxCollisionSpec valid_spec()
{
  return {
    "table",
    "robot_base",
    {1.5, 2.0, 0.05},
    {0.5, 0.0, -0.025},
  };
}

TEST(BoxCollision, BuildsMinimalMoveItWorldDiff)
{
  const auto scene =
    manipulation::scene::make_planning_scene_diff(valid_spec());

  ASSERT_TRUE(scene.is_diff);
  ASSERT_EQ(scene.world.collision_objects.size(), 1U);
  const auto & object = scene.world.collision_objects.front();
  EXPECT_EQ(object.id, "table");
  EXPECT_EQ(object.header.frame_id, "robot_base");
  EXPECT_EQ(object.operation, moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_DOUBLE_EQ(object.pose.position.x, 0.5);
  ASSERT_EQ(object.primitives.size(), 1U);
  ASSERT_EQ(object.primitives.front().dimensions.size(), 3U);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[2], 0.05);
}

TEST(BoxCollision, RejectsInvalidGeometry)
{
  auto spec = valid_spec();
  spec.dimensions[2] = 0.0;
  EXPECT_FALSE(manipulation::scene::is_valid(spec));
  EXPECT_THROW(
    manipulation::scene::make_box_collision_object(spec),
    std::invalid_argument);

  spec = valid_spec();
  spec.position[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(manipulation::scene::is_valid(spec));

  spec = valid_spec();
  spec.frame_id = "robot_base/";
  EXPECT_FALSE(manipulation::scene::is_valid(spec));
}

TEST(BoxCollision, ParsesExactlyThreeFiniteParameterValues)
{
  const auto values =
    manipulation::scene::parse_vector3({1.0, 2.0, 3.0}, "position");
  EXPECT_DOUBLE_EQ(values[0], 1.0);
  EXPECT_THROW(
    manipulation::scene::parse_vector3({1.0, 2.0}, "position"),
    std::invalid_argument);
  EXPECT_THROW(
    manipulation::scene::parse_vector3(
      {1.0, std::numeric_limits<double>::infinity(), 3.0}, "position"),
    std::invalid_argument);
}

}  // namespace
