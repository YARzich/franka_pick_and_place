// Responsibility: Defines backend-neutral inputs, commands, states, and outcomes for ServoToPose.

#ifndef MANIPULATION__SERVO__SERVO_TO_POSE_TYPES_HPP_
#define MANIPULATION__SERVO__SERVO_TO_POSE_TYPES_HPP_

#include <chrono>
#include <cstdint>

namespace manipulation::servo
{

/// State of one ServoToPose session independent from its transport or backend.
enum class ServoToPoseState : std::uint8_t
{
  kAwaitingResume,
  kTracking,
  kHalting,
  kPaused,
  kTerminal,
};

/// Safe-stop phase used to separate Servo deceleration from backend pause.
enum class ServoToPoseHaltPhase : std::uint8_t
{
  kNone,
  kWaitingForLowVelocity,
  kAwaitingBackendPause,
  kVerifyingFinalPose,
};

/// Single side effect requested from the backend after one deterministic step.
enum class ServoToPoseCommand : std::uint8_t
{
  kNone,
  kPublishTarget,
  kRequestPause,
  kRequestResume,
};

/// Backend pause state proved by the adapter, not inferred by this state machine.
enum class ServoBackendState : std::uint8_t
{
  kUnknown,
  kRunning,
  kPaused,
};

/// Backend safety classification after adapter-specific status mapping.
enum class ServoSafetyStatus : std::uint8_t
{
  kNominal,
  kAdvisory,
  kStopRequired,
};

/// First condition that required the target stream to stop.
enum class ServoToPoseStopCause : std::uint8_t
{
  kNone,
  kGoalReached,
  kPauseRequested,
  kCancelRequested,
  kGoalTimeout,
  kPoseSampleStale,
  kPoseSampleInvalid,
  kBackendSafetyStop,
};

/// Typed terminal reason reported only after bounded halt processing.
enum class ServoToPoseTerminalReason : std::uint8_t
{
  kNone,
  kSucceeded,
  kCanceled,
  kGoalTimeout,
  kPoseSampleStale,
  kPoseSampleInvalid,
  kBackendSafetyStop,
  kHaltTimeout,
  kFinalPoseOutsideTolerance,
  kFinalPoseSampleStale,
  kFinalPoseSampleInvalid,
};

/// Immutable policy for one state-machine instance.
struct ServoToPoseStateMachineConfig
{
  using Duration = std::chrono::steady_clock::duration;

  double position_tolerance_m{0.001};
  double orientation_tolerance_rad{0.01};
  double stopped_velocity_tolerance{0.01};
  std::uint32_t required_target_samples{3U};
  std::uint32_t required_stopped_samples{3U};
  Duration goal_timeout{std::chrono::seconds{30}};
  Duration halt_timeout{std::chrono::seconds{2}};
};

/** One pose-error observation produced by an external state estimator. */
struct ServoPoseSample
{
  /// Zero is reserved for "no sample" and is invalid when available is true.
  std::uint64_t sequence{0U};
  bool available{false};
  bool fresh{false};
  bool valid{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

/** One maximum-joint-velocity observation used to prove a completed halt. */
struct ServoVelocitySample
{
  /// Zero is reserved for "no sample" and is invalid when available is true.
  std::uint64_t sequence{0U};
  bool available{false};
  bool fresh{false};
  bool valid{false};
  double max_abs_joint_velocity{0.0};
};

/** Complete explicit input for one state-machine transition. */
struct ServoToPoseStepInput
{
  using Duration = std::chrono::steady_clock::duration;

  /// Monotonic time since this goal was accepted, supplied by the caller.
  Duration elapsed{};
  bool cancel_requested{false};
  bool pause_requested{false};
  ServoBackendState backend_state{ServoBackendState::kUnknown};
  ServoSafetyStatus safety_status{ServoSafetyStatus::kNominal};
  ServoPoseSample pose_sample;
  ServoVelocitySample velocity_sample;
};

/** Snapshot plus the one backend command selected by a transition. */
struct ServoToPoseStepResult
{
  ServoToPoseState state{ServoToPoseState::kAwaitingResume};
  ServoToPoseHaltPhase halt_phase{ServoToPoseHaltPhase::kNone};
  ServoToPoseCommand command{ServoToPoseCommand::kNone};
  ServoToPoseStopCause stop_cause{ServoToPoseStopCause::kNone};
  ServoToPoseTerminalReason terminal_reason{ServoToPoseTerminalReason::kNone};
  bool stop_confirmed{false};
  std::uint32_t consecutive_target_samples{0U};
  std::uint32_t consecutive_stopped_samples{0U};
  std::uint32_t consecutive_final_pose_samples{0U};

  [[nodiscard]] bool terminal() const noexcept
  {
    return state == ServoToPoseState::kTerminal;
  }
};

}  // namespace manipulation::servo

#endif  // MANIPULATION__SERVO__SERVO_TO_POSE_TYPES_HPP_
