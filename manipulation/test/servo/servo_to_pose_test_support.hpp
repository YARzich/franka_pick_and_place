// Responsibility: Builds concise deterministic inputs for ServoToPose state-machine tests.

#ifndef SERVO__SERVO_TO_POSE_TEST_SUPPORT_HPP_
#define SERVO__SERVO_TO_POSE_TEST_SUPPORT_HPP_

#include <chrono>
#include <cstdint>

#include "manipulation/servo/servo_to_pose_state_machine.hpp"

namespace manipulation::servo::test
{

inline ServoToPoseStateMachineConfig default_config()
{
  ServoToPoseStateMachineConfig config;
  config.position_tolerance_m = 0.01;
  config.orientation_tolerance_rad = 0.02;
  config.stopped_velocity_tolerance = 0.03;
  config.required_target_samples = 3U;
  config.required_stopped_samples = 2U;
  config.goal_timeout = std::chrono::seconds{10};
  config.halt_timeout = std::chrono::seconds{2};
  return config;
}

inline ServoPoseSample pose_sample(
  const std::uint64_t sequence,
  const double position_error_m = 0.1,
  const double orientation_error_rad = 0.1)
{
  ServoPoseSample sample;
  sample.sequence = sequence;
  sample.available = true;
  sample.fresh = true;
  sample.valid = true;
  sample.position_error_m = position_error_m;
  sample.orientation_error_rad = orientation_error_rad;
  return sample;
}

inline ServoVelocitySample velocity_sample(
  const std::uint64_t sequence,
  const double max_abs_joint_velocity = 0.0)
{
  ServoVelocitySample sample;
  sample.sequence = sequence;
  sample.available = true;
  sample.fresh = true;
  sample.valid = true;
  sample.max_abs_joint_velocity = max_abs_joint_velocity;
  return sample;
}

inline ServoToPoseStepInput input_at(const std::chrono::milliseconds elapsed)
{
  ServoToPoseStepInput input;
  input.elapsed = elapsed;
  input.backend_state = ServoBackendState::kRunning;
  input.safety_status = ServoSafetyStatus::kNominal;
  input.pose_sample = pose_sample(1U);
  return input;
}

inline ServoToPoseStepResult begin_tracking(
  ServoToPoseStateMachine & machine,
  ServoPoseSample first_pose = pose_sample(1U))
{
  auto input = input_at(std::chrono::milliseconds{0});
  input.backend_state = ServoBackendState::kPaused;
  const auto resume = machine.step(input);
  if (resume.command != ServoToPoseCommand::kRequestResume) {
    return resume;
  }

  input = input_at(std::chrono::milliseconds{1});
  input.pose_sample = first_pose;
  return machine.step(input);
}

inline ServoToPoseStepResult provide_confirmed_stop(
  ServoToPoseStateMachine & machine,
  const std::chrono::milliseconds first_elapsed,
  const std::uint64_t first_sequence = 1U)
{
  auto input = input_at(first_elapsed);
  ServoToPoseStepResult result;
  for (std::uint32_t index = 0U; index < machine.config().required_stopped_samples; ++index) {
    input.backend_state = ServoBackendState::kRunning;
    input.velocity_sample = velocity_sample(first_sequence + index);
    result = machine.step(input);
    input.elapsed += std::chrono::milliseconds{1};
  }

  input.backend_state = ServoBackendState::kPaused;
  input.velocity_sample = ServoVelocitySample{};
  return machine.step(input);
}

inline ServoToPoseStepResult provide_final_pose_proof(
  ServoToPoseStateMachine & machine,
  const std::chrono::milliseconds first_elapsed,
  const std::uint64_t first_sequence)
{
  auto input = input_at(first_elapsed);
  input.backend_state = ServoBackendState::kPaused;
  ServoToPoseStepResult result;
  for (std::uint32_t index = 0U; index < machine.config().required_target_samples; ++index) {
    input.pose_sample = pose_sample(first_sequence + index, 0.001, 0.001);
    result = machine.step(input);
    input.elapsed += std::chrono::milliseconds{1};
  }
  return result;
}

}  // namespace manipulation::servo::test

#endif  // SERVO__SERVO_TO_POSE_TEST_SUPPORT_HPP_
