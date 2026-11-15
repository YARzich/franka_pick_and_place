// Responsibility: Declares conversion from backend-neutral task state to stable ROS messages.

#ifndef MANIPULATION__WORKFLOW__TASK_MESSAGE_ADAPTER_HPP_
#define MANIPULATION__WORKFLOW__TASK_MESSAGE_ADAPTER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/duration.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation_msgs/msg/manipulation_task_result.hpp"
#include "manipulation_msgs/msg/manipulation_task_state.hpp"

namespace manipulation::workflow
{

/// Validated domain outcome and steady elapsed time decoded from a ROS result.
struct DecodedTaskResult
{
  manipulation::task::TaskOutcome outcome;
  std::chrono::steady_clock::duration elapsed{};
};

/// Non-throwing result of decoding an untrusted task-result message.
struct TaskResultDecodeResult
{
  std::optional<DecodedTaskResult> value;
  std::string reason;

  [[nodiscard]] bool valid() const noexcept
  {
    return value.has_value();
  }
};

/// Non-throwing result of decoding untrusted task feedback.
struct TaskStateDecodeResult
{
  std::optional<manipulation::task::TaskStateSnapshot> value;
  std::string reason;

  [[nodiscard]] bool valid() const noexcept
  {
    return value.has_value();
  }
};

/**
 * Convert an internal workflow phase to its stable ROS wire value.
 *
 * Every core enum value is mapped explicitly. The function throws
 * std::invalid_argument for an invalid enum representation; callers must not
 * reinterpret or persist the numeric value of the core enum directly.
 */
[[nodiscard]] std::uint8_t to_wire_phase(manipulation::task::TaskPhase phase);

/**
 * Convert an internal task error to its stable ROS wire value.
 *
 * Every core enum value is mapped explicitly. The function throws
 * std::invalid_argument for an invalid enum representation.
 */
[[nodiscard]] std::uint16_t to_wire_error_code(
  manipulation::task::TaskErrorCode error_code);

/**
 * Convert an internal object disposition to its stable ROS wire value.
 *
 * Every core enum value is mapped explicitly. The function throws
 * std::invalid_argument for an invalid enum representation.
 */
[[nodiscard]] std::uint8_t to_wire_disposition(
  manipulation::task::ObjectDisposition disposition);

/**
 * Convert steady elapsed time to a normalized ROS duration.
 *
 * Negative durations are clamped to zero. Values above the positive range of
 * builtin_interfaces/Duration are saturated to INT32_MAX seconds plus
 * 999,999,999 nanoseconds. The returned nanosecond field is always below 1e9.
 */
[[nodiscard]] builtin_interfaces::msg::Duration to_ros_duration(
  std::chrono::steady_clock::duration elapsed) noexcept;

/**
 * Serialize a backend-independent workflow snapshot for ROS action feedback.
 *
 * Finite progress is clamped to [0.0, 1.0]. NaN and negative infinity become
 * 0.0; positive infinity becomes 1.0. Other typed fields and operator detail
 * are copied without reinterpretation.
 */
[[nodiscard]] manipulation_msgs::msg::ManipulationTaskState to_ros_message(
  const manipulation::task::TaskStateSnapshot & snapshot);

/**
 * Serialize a terminal workflow outcome for a ROS action result.
 *
 * elapsed follows the same steady-time saturation policy as feedback. The
 * outcome detail remains operator-facing text; clients branch on error_code
 * and final_disposition.
 */
[[nodiscard]] manipulation_msgs::msg::ManipulationTaskResult to_ros_message(
  const manipulation::task::TaskOutcome & outcome,
  std::chrono::steady_clock::duration elapsed);

/**
 * Decode a task result while enforcing enum, success, and duration invariants.
 *
 * Unknown future enum values and inconsistent `success`/`error_code` pairs are
 * rejected instead of being silently reinterpreted. Callers decide whether an
 * invalid backend result maps to internal error or unknown external state.
 */
[[nodiscard]] TaskResultDecodeResult decode_task_result(
  const manipulation_msgs::msg::ManipulationTaskResult & message);

/// Decode task feedback while rejecting unknown phases and malformed values.
[[nodiscard]] TaskStateDecodeResult decode_task_state(
  const manipulation_msgs::msg::ManipulationTaskState & message);

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TASK_MESSAGE_ADAPTER_HPP_
