// Responsibility: Defines backend-neutral task phases, failure codes and terminal outcome values.

#ifndef MANIPULATION__TASK__TASK_STATUS_HPP_
#define MANIPULATION__TASK__TASK_STATUS_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace manipulation::task
{

/// Backend-independent phase of a manipulation workflow.
enum class TaskPhase : std::uint8_t
{
  kIdle,
  kValidating,
  kWaitingForResources,
  kPlanning,
  kApproaching,
  kGrasping,
  kAttaching,
  kLifting,
  kTransferring,
  kPlacing,
  kReleasing,
  kDetaching,
  kRetreating,
  kRecovering,
  kCompleted,
  kCanceled,
  kFailed,
  kTracking,
};

/// Machine-readable terminal error independent from a planner backend.
enum class TaskErrorCode : std::uint16_t
{
  kNone,
  kCanceled,
  kInvalidGoal,
  kResourceUnavailable,
  kRobotStateStale,
  kControllerUnavailable,
  kTfError,
  kPerceptionTimeout,
  kPlanningFailed,
  kExecutionFailed,
  kGripperFailed,
  kSceneConflict,
  kSceneFailed,
  kTimeout,
  kRecoveryFailed,
  kInternalError,
};

/// Best-known physical state of the manipulated object after task termination.
enum class ObjectDisposition : std::uint8_t
{
  kUnknown,
  kSafeIdle,
  kObjectAtSource,
  kHoldingObject,
  kObjectAtTarget,
  kSceneUncertain,
};

/// Whether command-resource ownership may be released after task termination.
enum class ResourceReleaseDisposition : std::uint8_t
{
  /// Every dispatched physical command reached a terminal, stop-safe boundary.
  kReleaseSafe,
  /// At least one physical command may still be active; retain ownership.
  kQuarantineRequired,
};

/**
 * Backend-independent action feedback before serialization to a ROS message.
 *
 * Producers keep progress monotonic in [0.0, 1.0]. elapsed uses a steady clock
 * and must not be computed from ROS time. `detail` is operator context only;
 * clients branch on the typed fields.
 */
struct TaskStateSnapshot
{
  TaskPhase phase{TaskPhase::kIdle};
  float progress{0.0F};
  std::uint16_t step_index{0U};
  std::uint16_t step_count{0U};
  std::uint16_t retry_count{0U};
  std::uint16_t recovery_count{0U};
  bool paused{false};
  bool pause_pending{false};
  std::chrono::steady_clock::duration elapsed{};
  std::string detail;
};

/**
 * Immutable terminal outcome with explicit physical object disposition.
 *
 * Use the factories so a successful result cannot carry an error and a failed
 * result cannot silently use kNone. `detail` is not a machine-readable API.
 */
class TaskOutcome final
{
public:
  /// Construct a successful terminal outcome.
  [[nodiscard]] static TaskOutcome make_success(
    ObjectDisposition disposition, std::string detail = {});

  /// Construct a success that followed bounded recovery attempts.
  [[nodiscard]] static TaskOutcome make_success(
    ObjectDisposition disposition, std::uint16_t recovery_count,
    std::string detail);

  /**
   * Construct a failed/canceled terminal outcome.
   *
   * Throws std::invalid_argument when error_code is kNone because that would
   * violate the outcome invariant.
   */
  [[nodiscard]] static TaskOutcome make_failure(
    TaskErrorCode error_code, ObjectDisposition disposition,
    std::uint16_t recovery_count = 0U, std::string detail = {},
    ResourceReleaseDisposition resource_release =
    ResourceReleaseDisposition::kReleaseSafe);

  /// Return true only for a success created by make_success().
  [[nodiscard]] bool succeeded() const noexcept;

  /// Return the stable error code; kNone denotes success.
  [[nodiscard]] TaskErrorCode error_code() const noexcept;

  /// Return the best-known physical object disposition at termination.
  [[nodiscard]] ObjectDisposition disposition() const noexcept;

  /// Return the number of recovery attempts performed by the workflow.
  [[nodiscard]] std::uint16_t recovery_count() const noexcept;

  /**
   * Return whether shared command resources may be released.
   *
   * A terminal action result is not a physical stop proof. Backends must use
   * kQuarantineRequired when an accepted controller goal has not produced a
   * terminal result, so the action adapter cannot admit overlapping commands.
   */
  [[nodiscard]] ResourceReleaseDisposition resource_release() const noexcept;

  /// Convenience predicate for the ordinary RAII-release path.
  [[nodiscard]] bool resources_release_safe() const noexcept;

  /// Return human-readable operator context.
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  TaskOutcome(
    TaskErrorCode error_code, ObjectDisposition disposition,
    std::uint16_t recovery_count, std::string detail,
    ResourceReleaseDisposition resource_release);

  TaskErrorCode error_code_;
  ObjectDisposition disposition_;
  std::uint16_t recovery_count_;
  std::string detail_;
  ResourceReleaseDisposition resource_release_;
};

/// Return true for phases after which the workflow cannot continue normally.
[[nodiscard]] bool is_terminal(TaskPhase phase) noexcept;

/// Return true when disposition requires recovery or operator attention.
[[nodiscard]] bool requires_attention(ObjectDisposition disposition) noexcept;

}  // namespace manipulation::task

#endif  // MANIPULATION__TASK__TASK_STATUS_HPP_
