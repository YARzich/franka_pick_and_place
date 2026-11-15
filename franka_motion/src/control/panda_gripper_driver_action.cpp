// Responsibility: Handles gripper goal admission, cancellation and terminal action ownership.

#include "panda_gripper_driver.hpp"

#include <exception>
#include <limits>
#include <memory>
#include <utility>

#include "franka_motion/gripper_utils.hpp"

namespace franka_motion::detail
{

rclcpp_action::GoalResponse PandaGripperDriver::handleGoal(
  const rclcpp_action::GoalUUID & goal_id,
  std::shared_ptr<const GripperAction::Goal> goal)
{
  if (!isValidGripperCommand(
      goal->command.position, goal->command.max_effort))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Отклоняю некорректный goal: width=%.3f, max_effort=%.3f",
      goal->command.position, goal->command.max_effort);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!positionOnlyGripperAcceptsMaxEffort(goal->command.max_effort)) {
    RCLCPP_ERROR(
      get_logger(),
      "Отклоняю goal до reservation: position-only topic %s "
      "поддерживает только max_effort=0.0, получено %.6g",
      command_topic_.c_str(), goal->command.max_effort);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal_reservation_.reserve(goal_id)) {
    RCLCPP_WARN(
      get_logger(),
      "Отклоняю новый goal: предыдущая команда еще выполняется");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PandaGripperDriver::handleCancel(
  const std::shared_ptr<GoalHandle> & goal_handle)
{
  if (!goal_reservation_.request_cancel(goal_handle->get_goal_id())) {
    return rclcpp_action::CancelResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Принят запрос отмены goal хватателя");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PandaGripperDriver::handleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  active_goal_ = goal_handle;
  const auto activation = goal_reservation_.activate(goal_handle->get_goal_id());
  if (activation == GripperGoalActivation::kInvalidReservation) {
    abortActiveGoal("Нарушен инвариант reservation для goal хватателя");
    return;
  }
  if (activation == GripperGoalActivation::kCancelBeforeStart) {
    auto result = makeResult(false, false);
    active_goal_->canceled(result);
    active_goal_.reset();
    goal_reservation_.release();
    RCLCPP_INFO(
      get_logger(),
      "Goal хватателя отменен до отправки команды контроллеру");
    return;
  }

  const double target_width = clampGripperWidth(
    goal_handle->get_goal()->command.position);
  goal_evaluator_->startGoal(target_width, SteadyClock::now(), last_state_);
  publishWidthCommand(target_width);
  RCLCPP_INFO(
    get_logger(), "Выполняю команду ширины %.3f м", target_width);
}

void PandaGripperDriver::abortActiveGoal(const char * reason)
{
  auto result = makeResult(false, false);
  active_goal_->abort(result);
  active_goal_.reset();
  goal_reservation_.release();
  RCLCPP_ERROR(get_logger(), "%s", reason);
}

void PandaGripperDriver::shutdownActiveGoal() noexcept
{
  monitor_timer_->cancel();
  auto goal_handle = std::move(active_goal_);
  goal_reservation_.release();
  if (!goal_handle) {
    return;
  }

  // ServerGoalHandle's destructor auto-cancels non-terminal goals. After ROS
  // context shutdown that callback can no longer publish into the server goal
  // map, so force the C goal state terminal while the node still owns both.
  try {
    const auto result = makeResult(false, false);
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "Finalizing gripper goal during shutdown: %s", error.what());
  }
}

std::shared_ptr<PandaGripperDriver::GripperAction::Result>
PandaGripperDriver::makeResult(bool reached_goal, bool stalled) const
{
  auto result = std::make_shared<GripperAction::Result>();
  result->position = last_state_.has_value() ?
    last_state_->width() : std::numeric_limits<double>::quiet_NaN();
  result->effort = last_state_.has_value() ?
    last_state_->effort.value_or(0.0) : 0.0;
  result->reached_goal = reached_goal;
  result->stalled = stalled;
  return result;
}

}  // namespace franka_motion::detail
