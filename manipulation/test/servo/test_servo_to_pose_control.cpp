// Responsibility: Verifies cancel, timeout, pause/resume, escalation, and halt bounds.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "servo_to_pose_test_support.hpp"

namespace
{

using manipulation::servo::ServoBackendState;
using manipulation::servo::ServoSafetyStatus;
using manipulation::servo::ServoToPoseCommand;
using manipulation::servo::ServoToPoseHaltPhase;
using manipulation::servo::ServoToPoseState;
using manipulation::servo::ServoToPoseStateMachine;
using manipulation::servo::ServoToPoseStopCause;
using manipulation::servo::ServoToPoseTerminalReason;
using manipulation::servo::test::begin_tracking;
using manipulation::servo::test::default_config;
using manipulation::servo::test::input_at;
using manipulation::servo::test::pose_sample;
using manipulation::servo::test::provide_confirmed_stop;
using manipulation::servo::test::velocity_sample;
using namespace std::chrono_literals;

TEST(ServoToPoseControl, CancelWaitsForConfirmedBackendPauseAndStop)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.cancel_requested = true;
  auto result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kWaitingForLowVelocity);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kCancelRequested);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 3ms;
  input.cancel_requested = true;
  input.backend_state = ServoBackendState::kRunning;
  input.velocity_sample = velocity_sample(1U);
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);

  result = provide_confirmed_stop(machine, 4ms, 2U);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseControl, GoalTimeoutAlsoUsesTheBoundedHaltPath)
{
  auto config = default_config();
  config.goal_timeout = 5ms;
  ServoToPoseStateMachine machine(config);
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(5ms);
  input.pose_sample = pose_sample(2U);
  auto result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kGoalTimeout);
  EXPECT_FALSE(result.terminal());

  result = provide_confirmed_stop(machine, 6ms);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kGoalTimeout);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseControl, HaltTimeoutReportsUnconfirmedStop)
{
  auto config = default_config();
  config.halt_timeout = 5ms;
  ServoToPoseStateMachine machine(config);
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.cancel_requested = true;
  ASSERT_EQ(machine.step(input).state, ServoToPoseState::kHalting);

  input.elapsed = 6ms;
  input.cancel_requested = false;
  input.backend_state = ServoBackendState::kUnknown;
  auto result = machine.step(input);
  EXPECT_FALSE(result.terminal());
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);

  input.elapsed = 7ms;
  result = machine.step(input);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kHaltTimeout);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kCancelRequested);
  EXPECT_FALSE(result.stop_confirmed);
}

TEST(ServoToPoseControl, PauseWaitsForLowVelocityAndHoldsUntilResumeConfirmation)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(
    begin_tracking(machine, pose_sample(1U, 0.001, 0.001)).state,
    ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pause_requested = true;
  auto result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kWaitingForLowVelocity);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kPauseRequested);

  input.elapsed = 3ms;
  input.backend_state = ServoBackendState::kPaused;
  input.velocity_sample = velocity_sample(1U);
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);

  input.elapsed = 4ms;
  input.backend_state = ServoBackendState::kRunning;
  input.velocity_sample = velocity_sample(1U);
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);
  EXPECT_FALSE(result.terminal());
  input.elapsed = 5ms;
  input.velocity_sample = velocity_sample(2U);
  result = machine.step(input);
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kAwaitingBackendPause);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestPause);
  EXPECT_FALSE(result.stop_confirmed);

  input.elapsed = 6ms;
  input.backend_state = ServoBackendState::kPaused;
  result = machine.step(input);
  ASSERT_EQ(result.state, ServoToPoseState::kPaused);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 7ms;
  input.velocity_sample = velocity_sample(3U);
  result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kPaused);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);

  input.elapsed = 8ms;
  input.pause_requested = false;
  result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kAwaitingResume);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestResume);
  EXPECT_EQ(result.consecutive_target_samples, 0U);
  EXPECT_EQ(result.consecutive_stopped_samples, 0U);
  EXPECT_FALSE(result.stop_confirmed);

  input.elapsed = 9ms;
  input.backend_state = ServoBackendState::kPaused;
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kAwaitingResume);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestResume);

  input.elapsed = 10ms;
  input.backend_state = ServoBackendState::kRunning;
  result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kTracking);
  EXPECT_EQ(result.command, ServoToPoseCommand::kPublishTarget);
  EXPECT_EQ(result.consecutive_target_samples, 1U);
}

TEST(ServoToPoseControl, CancelWhileAlreadyPausedTerminatesImmediatelyAndSafely)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pause_requested = true;
  ASSERT_EQ(machine.step(input).state, ServoToPoseState::kHalting);
  auto result = provide_confirmed_stop(machine, 3ms);
  ASSERT_EQ(result.state, ServoToPoseState::kPaused);

  input = input_at(6ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pause_requested = true;
  input.cancel_requested = true;
  result = machine.step(input);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
}

TEST(ServoToPoseControl, CancelBeforeFirstResumeConfirmsStopWithoutUnpausing)
{
  ServoToPoseStateMachine machine(default_config());
  auto input = input_at(0ms);
  input.backend_state = ServoBackendState::kPaused;
  input.cancel_requested = true;

  auto result = machine.step(input);
  ASSERT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);

  input.elapsed = 1ms;
  input.velocity_sample = velocity_sample(1U);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);

  input.elapsed = 2ms;
  input.velocity_sample = velocity_sample(2U);
  result = machine.step(input);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
}

TEST(ServoToPoseControl, PauseBeforeFirstResumeConfirmsPausedWithoutUnpausing)
{
  ServoToPoseStateMachine machine(default_config());
  auto input = input_at(0ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pause_requested = true;

  ASSERT_EQ(machine.step(input).state, ServoToPoseState::kHalting);
  input.elapsed = 1ms;
  input.velocity_sample = velocity_sample(1U);
  EXPECT_EQ(machine.step(input).consecutive_stopped_samples, 1U);
  input.elapsed = 2ms;
  input.velocity_sample = velocity_sample(2U);
  const auto result = machine.step(input);

  EXPECT_EQ(result.state, ServoToPoseState::kPaused);
  EXPECT_FALSE(result.terminal());
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
}

TEST(ServoToPoseControl, CancelOverridesPendingSuccessfulHalt)
{
  ServoToPoseStateMachine machine(default_config());
  auto result = begin_tracking(machine, pose_sample(1U, 0.001, 0.001));
  ASSERT_EQ(result.consecutive_target_samples, 1U);

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  (void)machine.step(input);
  input.elapsed = 3ms;
  input.pose_sample = pose_sample(3U, 0.001, 0.001);
  result = machine.step(input);
  ASSERT_EQ(result.stop_cause, ServoToPoseStopCause::kGoalReached);

  input.elapsed = 4ms;
  input.cancel_requested = true;
  input.backend_state = ServoBackendState::kRunning;
  input.velocity_sample = velocity_sample(1U);
  result = machine.step(input);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kCancelRequested);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 5ms;
  input.velocity_sample = velocity_sample(2U);
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestPause);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 6ms;
  input.backend_state = ServoBackendState::kPaused;
  result = machine.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kCanceled);
}

TEST(ServoToPoseControl, PriorityStopsOverridePendingFinalPoseSuccess)
{
  auto enter_verification = [](
    ServoToPoseStateMachine & machine) {
      (void)begin_tracking(machine, pose_sample(1U, 0.001, 0.001));
      auto input = input_at(2ms);
      input.pose_sample = pose_sample(2U, 0.001, 0.001);
      (void)machine.step(input);
      input.elapsed = 3ms;
      input.pose_sample = pose_sample(3U, 0.001, 0.001);
      (void)machine.step(input);
      return provide_confirmed_stop(machine, 4ms, 10U);
    };

  ServoToPoseStateMachine canceled(default_config());
  ASSERT_EQ(
    enter_verification(canceled).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);
  auto input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.cancel_requested = true;
  auto result = canceled.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);

  ServoToPoseStateMachine safety_stopped(default_config());
  ASSERT_EQ(
    enter_verification(safety_stopped).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);
  input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.safety_status = ServoSafetyStatus::kStopRequired;
  result = safety_stopped.step(input);
  EXPECT_EQ(
    result.terminal_reason,
    ServoToPoseTerminalReason::kBackendSafetyStop);
  EXPECT_TRUE(result.stop_confirmed);

  auto timeout_config = default_config();
  timeout_config.goal_timeout = 7ms;
  ServoToPoseStateMachine timed_out(timeout_config);
  ASSERT_EQ(
    enter_verification(timed_out).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);
  input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  result = timed_out.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kGoalTimeout);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseControl, HaltTimeoutAlsoBoundsBackendPauseConfirmation)
{
  auto config = default_config();
  config.halt_timeout = 5ms;
  ServoToPoseStateMachine machine(config);
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.cancel_requested = true;
  ASSERT_EQ(machine.step(input).state, ServoToPoseState::kHalting);

  input.elapsed = 3ms;
  input.velocity_sample = velocity_sample(1U);
  EXPECT_EQ(machine.step(input).command, ServoToPoseCommand::kNone);
  input.elapsed = 4ms;
  input.velocity_sample = velocity_sample(2U);
  auto result = machine.step(input);
  ASSERT_EQ(result.halt_phase, ServoToPoseHaltPhase::kAwaitingBackendPause);
  ASSERT_EQ(result.command, ServoToPoseCommand::kRequestPause);

  input.elapsed = 6ms;
  result = machine.step(input);
  EXPECT_FALSE(result.terminal());
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestPause);

  input.elapsed = 7ms;
  result = machine.step(input);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kAwaitingBackendPause);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kHaltTimeout);
  EXPECT_FALSE(result.stop_confirmed);
}

TEST(ServoToPoseControl, SevereStatusWhilePausedTerminatesWithConfirmedStop)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pause_requested = true;
  ASSERT_EQ(machine.step(input).state, ServoToPoseState::kHalting);
  ASSERT_EQ(provide_confirmed_stop(machine, 3ms).state, ServoToPoseState::kPaused);

  input = input_at(6ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pause_requested = true;
  input.safety_status = ServoSafetyStatus::kStopRequired;
  const auto result = machine.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kBackendSafetyStop);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseControl, RejectsNonMonotonicElapsedTime)
{
  ServoToPoseStateMachine machine(default_config());
  auto input = input_at(5ms);
  (void)machine.step(input);

  input.elapsed = 4ms;
  EXPECT_THROW(machine.step(input), std::invalid_argument);
}

}  // namespace
