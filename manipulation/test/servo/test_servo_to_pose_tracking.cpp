// Responsibility: Verifies target evidence, sample freshness, and tracking stop causes.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
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
using manipulation::servo::test::provide_final_pose_proof;
using manipulation::servo::test::velocity_sample;
using namespace std::chrono_literals;

TEST(ServoToPoseTracking, RejectsInvalidConfiguration)
{
  auto config = default_config();
  config.position_tolerance_m = 0.0;
  EXPECT_THROW(ServoToPoseStateMachine{config}, std::invalid_argument);

  config = default_config();
  config.orientation_tolerance_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ServoToPoseStateMachine{config}, std::invalid_argument);

  config = default_config();
  config.stopped_velocity_tolerance = -0.01;
  EXPECT_THROW(ServoToPoseStateMachine{config}, std::invalid_argument);

  config = default_config();
  config.required_target_samples = 0U;
  EXPECT_THROW(ServoToPoseStateMachine{config}, std::invalid_argument);

  config = default_config();
  config.halt_timeout = std::chrono::steady_clock::duration::zero();
  EXPECT_THROW(ServoToPoseStateMachine{config}, std::invalid_argument);
}

TEST(ServoToPoseTracking, RequiresExplicitResumeBeforePublishing)
{
  ServoToPoseStateMachine machine(default_config());
  auto input = input_at(0ms);

  const auto request = machine.step(input);
  EXPECT_EQ(request.state, ServoToPoseState::kAwaitingResume);
  EXPECT_EQ(request.command, ServoToPoseCommand::kRequestResume);

  input.elapsed = 1ms;
  input.backend_state = ServoBackendState::kUnknown;
  const auto waiting = machine.step(input);
  EXPECT_EQ(waiting.state, ServoToPoseState::kAwaitingResume);
  EXPECT_EQ(waiting.command, ServoToPoseCommand::kRequestResume);

  input.elapsed = 2ms;
  input.backend_state = ServoBackendState::kRunning;
  const auto tracking = machine.step(input);
  EXPECT_EQ(tracking.state, ServoToPoseState::kTracking);
  EXPECT_EQ(tracking.command, ServoToPoseCommand::kPublishTarget);
}

TEST(ServoToPoseTracking, CountsOnlyDistinctConsecutiveTargetSamples)
{
  ServoToPoseStateMachine machine(default_config());
  auto result = begin_tracking(machine, pose_sample(1U, 0.001, 0.001));
  ASSERT_EQ(result.consecutive_target_samples, 1U);

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(1U, 0.001, 0.001);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_target_samples, 1U);

  input.elapsed = 3ms;
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_target_samples, 2U);

  input.elapsed = 4ms;
  input.pose_sample = pose_sample(3U, 0.02, 0.001);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_target_samples, 0U);

  input.elapsed = 5ms;
  input.pose_sample = pose_sample(4U, 0.001, 0.001);
  EXPECT_EQ(machine.step(input).consecutive_target_samples, 1U);
  input.elapsed = 6ms;
  input.pose_sample = pose_sample(5U, 0.001, 0.001);
  EXPECT_EQ(machine.step(input).consecutive_target_samples, 2U);
  input.elapsed = 7ms;
  input.pose_sample = pose_sample(6U, 0.001, 0.001);
  result = machine.step(input);

  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kWaitingForLowVelocity);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kGoalReached);
  EXPECT_FALSE(result.terminal());
  EXPECT_FALSE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, RepeatedVelocitySamplesCannotProveAStop)
{
  ServoToPoseStateMachine machine(default_config());
  auto result = begin_tracking(machine, pose_sample(1U, 0.001, 0.001));

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  (void)machine.step(input);
  input.elapsed = 3ms;
  input.pose_sample = pose_sample(3U, 0.001, 0.001);
  result = machine.step(input);
  ASSERT_EQ(result.state, ServoToPoseState::kHalting);

  input.elapsed = 4ms;
  input.backend_state = ServoBackendState::kRunning;
  input.velocity_sample = velocity_sample(10U);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 5ms;
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_stopped_samples, 1U);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 6ms;
  input.velocity_sample = velocity_sample(11U);
  input.velocity_sample.fresh = false;
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_stopped_samples, 0U);

  input.elapsed = 7ms;
  input.velocity_sample = velocity_sample(11U);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_stopped_samples, 0U);
  EXPECT_FALSE(result.terminal());

  input.elapsed = 8ms;
  input.velocity_sample = velocity_sample(12U);
  EXPECT_EQ(machine.step(input).consecutive_stopped_samples, 1U);
  input.elapsed = 9ms;
  input.velocity_sample = velocity_sample(13U);
  result = machine.step(input);

  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kAwaitingBackendPause);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestPause);
  EXPECT_FALSE(result.terminal());
  EXPECT_FALSE(result.stop_confirmed);

  input.elapsed = 10ms;
  input.backend_state = ServoBackendState::kRunning;
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kRequestPause);

  input.elapsed = 11ms;
  input.backend_state = ServoBackendState::kPaused;
  result = machine.step(input);

  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.halt_phase, ServoToPoseHaltPhase::kVerifyingFinalPose);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_FALSE(result.terminal());

  result = provide_final_pose_proof(machine, 12ms, 20U);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kSucceeded);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, SuccessRequiresDistinctFinalPoseSamplesAfterPause)
{
  ServoToPoseStateMachine machine(default_config());
  auto result = begin_tracking(machine, pose_sample(1U, 0.001, 0.001));

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  (void)machine.step(input);
  input.elapsed = 3ms;
  input.pose_sample = pose_sample(3U, 0.001, 0.001);
  ASSERT_EQ(machine.step(input).stop_cause, ServoToPoseStopCause::kGoalReached);

  result = provide_confirmed_stop(machine, 4ms, 10U);
  ASSERT_EQ(result.halt_phase, ServoToPoseHaltPhase::kVerifyingFinalPose);
  ASSERT_TRUE(result.stop_confirmed);

  input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pose_sample = pose_sample(1U, 0.001, 0.001);
  result = machine.step(input);
  EXPECT_FALSE(result.terminal());
  EXPECT_EQ(result.consecutive_final_pose_samples, 0U);

  input.elapsed = 8ms;
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_final_pose_samples, 1U);
  input.elapsed = 9ms;
  result = machine.step(input);
  EXPECT_EQ(result.consecutive_final_pose_samples, 1U);

  input.elapsed = 10ms;
  input.pose_sample = pose_sample(3U, 0.001, 0.001);
  EXPECT_EQ(machine.step(input).consecutive_final_pose_samples, 2U);
  input.elapsed = 11ms;
  input.pose_sample = pose_sample(4U, 0.001, 0.001);
  result = machine.step(input);

  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kSucceeded);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, FinalPoseOutsideToleranceIsTypedSafeFailure)
{
  ServoToPoseStateMachine machine(default_config());
  (void)begin_tracking(machine, pose_sample(1U, 0.001, 0.001));

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  (void)machine.step(input);
  input.elapsed = 3ms;
  input.pose_sample = pose_sample(3U, 0.001, 0.001);
  ASSERT_EQ(machine.step(input).stop_cause, ServoToPoseStopCause::kGoalReached);
  ASSERT_EQ(
    provide_confirmed_stop(machine, 4ms, 10U).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);

  input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pose_sample = pose_sample(2U, 0.02, 0.001);
  const auto result = machine.step(input);

  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(
    result.terminal_reason,
    ServoToPoseTerminalReason::kFinalPoseOutsideTolerance);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, InvalidAndStaleFinalPoseAreTypedSafeFailures)
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

  ServoToPoseStateMachine stale_machine(default_config());
  ASSERT_EQ(
    enter_verification(stale_machine).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);
  auto input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  input.pose_sample.fresh = false;
  auto result = stale_machine.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kFinalPoseSampleStale);
  EXPECT_TRUE(result.stop_confirmed);

  ServoToPoseStateMachine invalid_machine(default_config());
  ASSERT_EQ(
    enter_verification(invalid_machine).halt_phase,
    ServoToPoseHaltPhase::kVerifyingFinalPose);
  input = input_at(7ms);
  input.backend_state = ServoBackendState::kPaused;
  input.pose_sample = pose_sample(2U, 0.001, 0.001);
  input.pose_sample.valid = false;
  result = invalid_machine.step(input);
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kFinalPoseSampleInvalid);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, StalePoseStopsStreamingBeforeTerminalResult)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pose_sample.fresh = false;
  auto result = machine.step(input);
  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kPoseSampleStale);
  EXPECT_FALSE(result.terminal());

  result = provide_confirmed_stop(machine, 3ms);
  EXPECT_TRUE(result.terminal());
  EXPECT_EQ(result.terminal_reason, ServoToPoseTerminalReason::kPoseSampleStale);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTracking, InvalidOrRegressingPoseSampleFailsClosed)
{
  ServoToPoseStateMachine invalid_machine(default_config());
  ASSERT_EQ(begin_tracking(invalid_machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(
    2U, std::numeric_limits<double>::quiet_NaN(), 0.1);
  auto result = invalid_machine.step(input);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kPoseSampleInvalid);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);

  ServoToPoseStateMachine regressing_machine(default_config());
  ASSERT_EQ(
    begin_tracking(regressing_machine, pose_sample(5U)).state,
    ServoToPoseState::kTracking);
  input.elapsed = 2ms;
  input.pose_sample = pose_sample(4U);
  result = regressing_machine.step(input);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kPoseSampleInvalid);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
}

TEST(ServoToPoseTracking, AdvisoryContinuesButSevereStatusRequestsHalt)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.pose_sample = pose_sample(2U);
  input.safety_status = ServoSafetyStatus::kAdvisory;
  auto result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kPublishTarget);

  input.elapsed = 3ms;
  input.safety_status = ServoSafetyStatus::kStopRequired;
  result = machine.step(input);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kBackendSafetyStop);
  EXPECT_FALSE(result.terminal());
}

TEST(ServoToPoseTracking, LosingConfirmedRunningStateStopsTargetPublication)
{
  ServoToPoseStateMachine machine(default_config());
  ASSERT_EQ(begin_tracking(machine).state, ServoToPoseState::kTracking);

  auto input = input_at(2ms);
  input.backend_state = ServoBackendState::kUnknown;
  input.pose_sample = pose_sample(2U);
  const auto result = machine.step(input);

  EXPECT_EQ(result.state, ServoToPoseState::kHalting);
  EXPECT_EQ(result.command, ServoToPoseCommand::kNone);
  EXPECT_EQ(result.stop_cause, ServoToPoseStopCause::kBackendSafetyStop);
}

}  // namespace
