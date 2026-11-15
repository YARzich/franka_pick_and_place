// Responsibility: Declares internal ServoToPose result and feedback mapping policy.

#ifndef SERVO__SERVO_TO_POSE_TASK_POLICY_HPP_
#define SERVO__SERVO_TO_POSE_TASK_POLICY_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "manipulation/servo/servo_to_pose_state_machine.hpp"
#include "manipulation_moveit/servo/servo_to_pose_task.hpp"

namespace manipulation::moveit::servo_to_pose_detail
{

using Clock = ServoToPoseTask::Clock;

struct PendingFailure
{
  manipulation::task::TaskErrorCode error_code{
    manipulation::task::TaskErrorCode::kInternalError};
  std::string detail;
};

[[nodiscard]] Clock::time_point bounded_deadline(
  Clock::time_point absolute_deadline,
  Clock::duration operation_timeout) noexcept;

[[nodiscard]] Clock::duration nonnegative_elapsed(Clock::time_point started_at) noexcept;

[[nodiscard]] manipulation::task::TaskErrorCode setup_error_code(
  MoveItServoRosError error,
  bool target_preparation) noexcept;

[[nodiscard]] PendingFailure observation_failure(const MoveItServoRosResult & status);

[[nodiscard]] bool startup_feedback_ready(
  const MoveItServoObservationResult & observation) noexcept;

void emit_feedback(
  const ServoToPoseTask::FeedbackCallback & callback,
  const ServoToPoseTaskFeedback & feedback) noexcept;

[[nodiscard]] ServoToPoseTaskFeedback make_feedback(
  const manipulation::servo::ServoToPoseStepResult & step,
  const MoveItServoObservationResult & observation,
  const ServoToPoseTaskControl::Snapshot & control,
  Clock::duration elapsed);

[[nodiscard]] manipulation::task::TaskErrorCode terminal_error_code(
  const manipulation::servo::ServoToPoseStepResult & step,
  const std::optional<PendingFailure> & pending_failure) noexcept;

[[nodiscard]] std::string terminal_detail(
  const manipulation::servo::ServoToPoseStepResult & step,
  const std::optional<PendingFailure> & pending_failure);

}  // namespace manipulation::moveit::servo_to_pose_detail

#endif  // SERVO__SERVO_TO_POSE_TASK_POLICY_HPP_
