// Responsibility: Verifies gripper completion, contact, timeout, and hold decisions.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <optional>
#include <utility>

#include "franka_motion/gripper_goal_evaluator.hpp"

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Transition = franka_motion::GripperGoalTransition;

franka_motion::GripperGoalPolicy test_policy()
{
  franka_motion::GripperGoalPolicy policy;
  policy.goal_tolerance = 0.001;
  policy.movement_threshold = 0.0005;
  policy.finger_symmetry_tolerance = 0.003;
  policy.maximum_reverse_motion = 0.002;
  policy.allow_stalling = true;
  policy.state_timeout = 200ms;
  policy.stall_timeout = 100ms;
  policy.goal_timeout = 1s;
  return policy;
}

franka_motion::GripperFingerState finger_state(
  double left,
  double right,
  Clock::time_point received_at)
{
  franka_motion::GripperFingerState state;
  state.left_joint = left;
  state.right_joint = right;
  state.received_at = received_at;
  return state;
}

franka_motion::GripperGoalEvaluation evaluate_contact_stall(
  double current_left,
  double current_right,
  bool allow_stalling = true)
{
  const auto start = Clock::time_point{};
  auto policy = test_policy();
  policy.allow_stalling = allow_stalling;
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.0, start, finger_state(0.04, 0.04, start));

  EXPECT_EQ(
    evaluator.evaluate(start + 1ms, finger_state(0.04, 0.04, start + 1ms)).transition,
    Transition::kContinue);
  EXPECT_EQ(
    evaluator.evaluate(
      start + 10ms,
      finger_state(current_left, current_right, start + 10ms)).transition,
    Transition::kContinue);
  return evaluator.evaluate(
    start + 110ms,
    finger_state(current_left, current_right, start + 110ms));
}

}  // namespace

TEST(GripperGoalEvaluator, RejectsInvalidPolicyAndTarget)
{
  auto policy = test_policy();
  policy.goal_tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    (void)franka_motion::GripperGoalEvaluator{policy},
    std::invalid_argument);

  policy = test_policy();
  franka_motion::GripperGoalEvaluator evaluator(policy);
  EXPECT_THROW(
    evaluator.startGoal(0.081, Clock::time_point{}),
    std::invalid_argument);
}

TEST(GripperGoalEvaluator, RequiresFreshPostGoalState)
{
  const auto start = Clock::time_point{} + 1s;
  auto policy = test_policy();
  policy.state_timeout = 100ms;
  policy.stall_timeout = 1s;
  policy.goal_timeout = 2s;
  const auto cached_state = finger_state(0.02, 0.02, start - 50ms);
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.06, start, cached_state);

  const auto waiting = evaluator.evaluate(start + 99ms, cached_state);
  EXPECT_EQ(waiting.transition, Transition::kContinue);
  EXPECT_FALSE(waiting.publish_feedback);

  const auto timed_out = evaluator.evaluate(start + 100ms, cached_state);
  EXPECT_EQ(timed_out.transition, Transition::kStateTimeout);
  EXPECT_TRUE(timed_out.request_hold);
}

TEST(GripperGoalEvaluator, TreatsExactStateAgeBoundaryAsFresh)
{
  const auto start = Clock::time_point{} + 1s;
  auto policy = test_policy();
  policy.state_timeout = 100ms;
  policy.stall_timeout = 1s;
  policy.goal_timeout = 2s;
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.06, start);
  const auto state = finger_state(0.02, 0.02, start + 1ms);

  const auto exact_boundary = evaluator.evaluate(start + 101ms, state);
  EXPECT_EQ(exact_boundary.transition, Transition::kContinue);
  EXPECT_TRUE(exact_boundary.publish_feedback);

  const auto stale = evaluator.evaluate(start + 102ms, state);
  EXPECT_EQ(stale.transition, Transition::kStateTimeout);
}

TEST(GripperGoalEvaluator, RequiresWidthAndFingerSymmetryToReachGoal)
{
  const auto start = Clock::time_point{};
  const auto policy = test_policy();

  for (const double direction : {-1.0, 1.0}) {
    franka_motion::GripperGoalEvaluator evaluator(policy);
    evaluator.startGoal(0.04, start);
    const auto at_tolerance = finger_state(
      0.02 + direction * 0.001,
      0.02 - direction * 0.001,
      start + 1ms);
    const auto reached = evaluator.evaluate(start + 1ms, at_tolerance);
    EXPECT_EQ(reached.transition, Transition::kReached);
    EXPECT_TRUE(reached.feedback_reached);
  }

  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.04, start);
  const auto asymmetric = evaluator.evaluate(
    start + 1ms,
    finger_state(0.022, 0.018, start + 1ms));
  EXPECT_EQ(asymmetric.transition, Transition::kContinue);
  EXPECT_FALSE(asymmetric.feedback_reached);
}

TEST(GripperGoalEvaluator, AcceptsMirroredOneSidedContactStalls)
{
  for (const auto & state : {
    std::pair<double, double>{0.04, 0.0},
    std::pair<double, double>{0.0, 0.04}})
  {
    const auto evaluation = evaluate_contact_stall(state.first, state.second);
    EXPECT_EQ(evaluation.transition, Transition::kStallSucceeded);
    EXPECT_TRUE(evaluation.result_stalled);
    EXPECT_TRUE(evaluation.closing_progress);
    EXPECT_TRUE(evaluation.finger_motion_supports_contact);
  }
}

TEST(GripperGoalEvaluator, AppliesReverseMotionToleranceSymmetrically)
{
  for (const double direction : {-1.0, 1.0}) {
    const double allowed_left = direction > 0.0 ? 0.0419 : 0.0;
    const double allowed_right = direction > 0.0 ? 0.0 : 0.0419;
    const auto allowed = evaluate_contact_stall(allowed_left, allowed_right);
    EXPECT_EQ(allowed.transition, Transition::kStallSucceeded);

    const double rejected_left = direction > 0.0 ? 0.0421 : 0.0;
    const double rejected_right = direction > 0.0 ? 0.0 : 0.0421;
    const auto rejected = evaluate_contact_stall(rejected_left, rejected_right);
    EXPECT_EQ(rejected.transition, Transition::kStallAborted);
    EXPECT_FALSE(rejected.result_stalled);
    EXPECT_FALSE(rejected.finger_motion_supports_contact);
  }
}

TEST(GripperGoalEvaluator, ReportsContactWhenStallingPolicyRequiresAbort)
{
  const auto evaluation = evaluate_contact_stall(0.04, 0.0, false);
  EXPECT_EQ(evaluation.transition, Transition::kStallAborted);
  EXPECT_TRUE(evaluation.result_stalled);
  EXPECT_TRUE(evaluation.closing_progress);
}

TEST(GripperGoalEvaluator, RejectsStallsWithoutClosingProgress)
{
  const auto start = Clock::time_point{};
  const auto policy = test_policy();
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.0, start, finger_state(0.04, 0.04, start));
  EXPECT_EQ(
    evaluator.evaluate(start + 1ms, finger_state(0.04, 0.04, start + 1ms)).transition,
    Transition::kContinue);

  const auto stalled = evaluator.evaluate(
    start + 101ms,
    finger_state(0.04, 0.04, start + 101ms));
  EXPECT_EQ(stalled.transition, Transition::kStallAborted);
  EXPECT_FALSE(stalled.result_stalled);
  EXPECT_FALSE(stalled.closing_progress);
}

TEST(GripperGoalEvaluator, PreservesTerminalTransitionPriority)
{
  const auto start = Clock::time_point{};
  auto policy = test_policy();
  policy.state_timeout = 100ms;
  policy.stall_timeout = 100ms;
  policy.goal_timeout = 100ms;

  franka_motion::GripperGoalEvaluator canceled(policy);
  canceled.startGoal(0.0, start, finger_state(0.04, 0.04, start));
  EXPECT_EQ(
    canceled.evaluate(start + 100ms, std::nullopt, true).transition,
    Transition::kCanceled);

  franka_motion::GripperGoalEvaluator stale(policy);
  stale.startGoal(0.0, start);
  EXPECT_EQ(
    stale.evaluate(start + 100ms, std::nullopt).transition,
    Transition::kStateTimeout);

  franka_motion::GripperGoalEvaluator reached(policy);
  reached.startGoal(0.08, start, finger_state(0.0, 0.0, start));
  const auto reached_state = finger_state(0.04, 0.04, start + 100ms);
  EXPECT_EQ(
    reached.evaluate(start + 100ms, reached_state).transition,
    Transition::kReached);

  franka_motion::GripperGoalEvaluator stalled(policy);
  stalled.startGoal(0.0, start, finger_state(0.04, 0.04, start));
  EXPECT_EQ(
    stalled.evaluate(start, finger_state(0.04, 0.04, start)).transition,
    Transition::kContinue);
  EXPECT_EQ(
    stalled.evaluate(start + 1ms, finger_state(0.02, 0.02, start + 1ms)).transition,
    Transition::kContinue);
  EXPECT_EQ(
    stalled.evaluate(
      start + 101ms,
      finger_state(0.02, 0.02, start + 101ms)).transition,
    Transition::kStallSucceeded);
}

TEST(GripperGoalEvaluator, NewMovementClearsStallBeforeFeedback)
{
  const auto start = Clock::time_point{};
  const auto policy = test_policy();
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.0, start, finger_state(0.04, 0.04, start));
  EXPECT_EQ(
    evaluator.evaluate(start, finger_state(0.04, 0.04, start)).transition,
    Transition::kContinue);

  const auto resumed = evaluator.evaluate(
    start + 100ms,
    finger_state(0.02, 0.02, start + 100ms));
  EXPECT_EQ(resumed.transition, Transition::kContinue);
  EXPECT_TRUE(resumed.publish_feedback);
  EXPECT_FALSE(resumed.feedback_stalled);
}

TEST(GripperGoalEvaluator, ReachedFeedbackCannotAlsoReportStalled)
{
  const auto start = Clock::time_point{};
  auto policy = test_policy();
  policy.goal_tolerance = 0.0005;
  policy.movement_threshold = 0.001;
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.04, start);
  EXPECT_EQ(
    evaluator.evaluate(start, finger_state(0.0203, 0.0203, start)).transition,
    Transition::kContinue);

  const auto reached = evaluator.evaluate(
    start + 100ms,
    finger_state(0.02, 0.02, start + 100ms));
  EXPECT_EQ(reached.transition, Transition::kReached);
  EXPECT_TRUE(reached.feedback_reached);
  EXPECT_FALSE(reached.feedback_stalled);
  EXPECT_TRUE(reached.result_reached);
  EXPECT_FALSE(reached.result_stalled);
}

TEST(GripperGoalEvaluator, GoalTimeoutRequiresFreshNonStalledState)
{
  const auto start = Clock::time_point{};
  auto policy = test_policy();
  policy.state_timeout = 200ms;
  policy.stall_timeout = 60ms;
  policy.goal_timeout = 100ms;
  franka_motion::GripperGoalEvaluator evaluator(policy);
  evaluator.startGoal(0.07, start, finger_state(0.005, 0.005, start));

  EXPECT_EQ(
    evaluator.evaluate(start, finger_state(0.005, 0.005, start)).transition,
    Transition::kContinue);
  EXPECT_EQ(
    evaluator.evaluate(start + 60ms, finger_state(0.01, 0.01, start + 60ms)).transition,
    Transition::kContinue);
  const auto timed_out = evaluator.evaluate(
    start + 100ms,
    finger_state(0.015, 0.015, start + 100ms));
  EXPECT_EQ(timed_out.transition, Transition::kGoalTimeout);
  EXPECT_TRUE(timed_out.request_hold);
}
