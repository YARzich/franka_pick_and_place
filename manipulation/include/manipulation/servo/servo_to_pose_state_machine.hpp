// Responsibility: Selects safe ServoToPose commands from explicit observations and control intent.

#ifndef MANIPULATION__SERVO__SERVO_TO_POSE_STATE_MACHINE_HPP_
#define MANIPULATION__SERVO__SERVO_TO_POSE_STATE_MACHINE_HPP_

#include <cstdint>

#include "manipulation/servo/servo_to_pose_types.hpp"

namespace manipulation::servo
{

/**
 * Pure deterministic state machine for one reactive pose-tracking session.
 *
 * The class does not read clocks, publish commands, call services, or know ROS,
 * MoveIt, robot frames, or controller names. The caller supplies monotonic
 * elapsed time and backend-confirmed observations. A returned command is the
 * only side effect requested for that transition.
 *
 * Goal completion, cancellation, timeout, stale input, invalid input, and a
 * severe backend status all stop target publication first. The machine lets
 * the still-running backend decelerate, proves low velocity using distinct
 * fresh samples, and only then requests backend pause. A terminal result is
 * emitted after that pause is confirmed, or after the bounded halt timeout.
 * A successful goal additionally requires new, distinct pose samples proving
 * that the already-stopped TCP remained inside the requested tolerance.
 */
class ServoToPoseStateMachine final
{
public:
  explicit ServoToPoseStateMachine(ServoToPoseStateMachineConfig config);

  ServoToPoseStateMachine(const ServoToPoseStateMachine &) = delete;
  ServoToPoseStateMachine & operator=(const ServoToPoseStateMachine &) = delete;
  ServoToPoseStateMachine(ServoToPoseStateMachine &&) = delete;
  ServoToPoseStateMachine & operator=(ServoToPoseStateMachine &&) = delete;

  /// Advance exactly once from caller-supplied state and return one next command.
  [[nodiscard]] ServoToPoseStepResult step(const ServoToPoseStepInput & input);

  /// Return the current state without advancing or requesting a side effect.
  [[nodiscard]] ServoToPoseStepResult snapshot() const noexcept;

  [[nodiscard]] const ServoToPoseStateMachineConfig & config() const noexcept;

private:
  using Duration = ServoToPoseStateMachineConfig::Duration;

  [[nodiscard]] ServoToPoseStepResult make_result(ServoToPoseCommand command) const noexcept;
  [[nodiscard]] ServoToPoseStepResult step_awaiting_resume(
    const ServoToPoseStepInput & input);
  [[nodiscard]] ServoToPoseStepResult step_tracking(
    const ServoToPoseStepInput & input);
  [[nodiscard]] ServoToPoseStepResult step_halting(
    const ServoToPoseStepInput & input);
  [[nodiscard]] ServoToPoseStepResult step_verifying_final_pose(
    const ServoToPoseStepInput & input);
  [[nodiscard]] ServoToPoseStepResult step_paused(
    const ServoToPoseStepInput & input);

  [[nodiscard]] bool handle_priority_stop(const ServoToPoseStepInput & input);
  [[nodiscard]] ServoToPoseStepResult begin_halt(
    ServoToPoseStopCause cause, Duration elapsed);
  [[nodiscard]] ServoToPoseStepResult complete_confirmed_halt(
    const ServoToPoseStepInput & input);
  void escalate_stop_cause(ServoToPoseStopCause cause) noexcept;
  void enter_tracking() noexcept;
  void enter_final_pose_verification(const ServoPoseSample & baseline) noexcept;
  void enter_terminal(ServoToPoseTerminalReason reason, bool stop_confirmed) noexcept;
  void reset_target_evidence() noexcept;
  void reset_stop_evidence() noexcept;
  void reset_final_pose_evidence() noexcept;

  [[nodiscard]] ServoToPoseTerminalReason terminal_reason_for_stop_cause() const noexcept;
  [[nodiscard]] bool pose_sample_is_new(const ServoPoseSample & sample) noexcept;
  [[nodiscard]] bool velocity_sample_is_new(const ServoVelocitySample & sample) noexcept;
  [[nodiscard]] bool final_pose_sample_is_new(const ServoPoseSample & sample) noexcept;

  const ServoToPoseStateMachineConfig config_;
  ServoToPoseState state_{ServoToPoseState::kAwaitingResume};
  ServoToPoseHaltPhase halt_phase_{ServoToPoseHaltPhase::kNone};
  ServoToPoseStopCause stop_cause_{ServoToPoseStopCause::kNone};
  ServoToPoseTerminalReason terminal_reason_{ServoToPoseTerminalReason::kNone};
  bool stop_confirmed_{false};
  bool resume_command_issued_{false};
  bool elapsed_initialized_{false};
  Duration last_elapsed_{};
  Duration halt_started_at_{};
  std::uint32_t consecutive_target_samples_{0U};
  std::uint32_t consecutive_stopped_samples_{0U};
  std::uint32_t consecutive_final_pose_samples_{0U};
  bool pose_sequence_initialized_{false};
  bool velocity_sequence_initialized_{false};
  bool final_pose_sequence_initialized_{false};
  std::uint64_t last_pose_sequence_{0U};
  std::uint64_t last_velocity_sequence_{0U};
  std::uint64_t last_final_pose_sequence_{0U};
};

}  // namespace manipulation::servo

#endif  // MANIPULATION__SERVO__SERVO_TO_POSE_STATE_MACHINE_HPP_
