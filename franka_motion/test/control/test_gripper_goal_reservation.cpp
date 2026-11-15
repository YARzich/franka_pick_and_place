// Responsibility: Verifies exclusive gripper-goal ownership and goal-specific cancellation races.

#include <gtest/gtest.h>

#include "franka_motion/gripper_goal_reservation.hpp"

namespace
{

using franka_motion::GripperGoalActivation;
using franka_motion::GripperGoalReservation;

GripperGoalReservation::GoalId goal_id(std::uint8_t marker)
{
  GripperGoalReservation::GoalId value{};
  value.front() = marker;
  return value;
}

TEST(GripperGoalReservation, RejectsConcurrentAndMismatchedGoals)
{
  GripperGoalReservation reservation;
  const auto first = goal_id(1U);
  const auto second = goal_id(2U);

  EXPECT_TRUE(reservation.reserve(first));
  EXPECT_TRUE(reservation.occupied());
  EXPECT_FALSE(reservation.reserve(second));
  EXPECT_FALSE(reservation.request_cancel(second));
  EXPECT_EQ(
    reservation.activate(second),
    GripperGoalActivation::kInvalidReservation);
  EXPECT_EQ(reservation.activate(first), GripperGoalActivation::kStart);
}

TEST(GripperGoalReservation, ReplaysCancelBeforePhysicalCommandStart)
{
  GripperGoalReservation reservation;
  const auto goal = goal_id(3U);

  ASSERT_TRUE(reservation.reserve(goal));
  ASSERT_TRUE(reservation.request_cancel(goal));
  EXPECT_EQ(
    reservation.activate(goal),
    GripperGoalActivation::kCancelBeforeStart);

  reservation.release();
  EXPECT_FALSE(reservation.occupied());
  EXPECT_TRUE(reservation.reserve(goal_id(4U)));
}

TEST(GripperGoalReservation, AcceptsCancelForTheActiveGoalOnly)
{
  GripperGoalReservation reservation;
  const auto goal = goal_id(5U);

  ASSERT_TRUE(reservation.reserve(goal));
  ASSERT_EQ(reservation.activate(goal), GripperGoalActivation::kStart);
  EXPECT_TRUE(reservation.request_cancel(goal));
  EXPECT_FALSE(reservation.request_cancel(goal_id(6U)));
}

}  // namespace
