// Responsibility: Converts joint feedback into goal progress and publishes controller commands.

#include "panda_gripper_driver.hpp"

#include <limits>
#include <memory>

#include "franka_motion/panda_joint_state_utils.hpp"
#include "panda_gripper_commands.hpp"

namespace franka_motion::detail
{

void PandaGripperDriver::handleJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto sample = parsePandaFingerJointState(*msg);
  if (!sample.has_value()) {
    return;
  }

  GripperFingerState state;
  state.left_joint = sample->left_position;
  state.right_joint = sample->right_position;
  state.received_at = SteadyClock::now();
  state.effort = sample->absolute_effort_sum;
  last_state_ = state;
}

void PandaGripperDriver::monitorActiveGoal()
{
  if (!active_goal_) {
    return;
  }
  if (!rclcpp::ok()) {
    shutdownActiveGoal();
    return;
  }

  const auto now = SteadyClock::now();
  const auto evaluation = goal_evaluator_->evaluate(
    now, last_state_, active_goal_->is_canceling());
  if (evaluation.publish_feedback) {
    publishFeedback(evaluation);
  }
  applyGoalEvaluation(evaluation);
}

void PandaGripperDriver::applyGoalEvaluation(
  const GripperGoalEvaluation & evaluation)
{
  using Transition = GripperGoalTransition;
  if (evaluation.request_hold) {
    publishHoldPosition();
  }

  switch (evaluation.transition) {
    case Transition::kInactive:
    case Transition::kContinue:
      return;
    case Transition::kCanceled:
      {
        auto result = makeResult(
          evaluation.result_reached, evaluation.result_stalled);
        active_goal_->canceled(result);
        active_goal_.reset();
        goal_reservation_.release();
        RCLCPP_INFO(
          get_logger(),
          "Goal хватателя отменен, удерживаем текущую позицию");
        return;
      }
    case Transition::kStateTimeout:
      abortActiveGoal("Нет свежего finger joint state");
      return;
    case Transition::kReached:
      {
        auto result = makeResult(
          evaluation.result_reached, evaluation.result_stalled);
        active_goal_->succeed(result);
        active_goal_.reset();
        goal_reservation_.release();
        RCLCPP_INFO(
          get_logger(),
          "Хвататель достиг ширины %.3f м",
          result->position);
        return;
      }
    case Transition::kStallSucceeded:
      {
        auto result = makeResult(
          evaluation.result_reached, evaluation.result_stalled);
        active_goal_->succeed(result);
        active_goal_.reset();
        goal_reservation_.release();
        RCLCPP_INFO(
          get_logger(),
          "Хвататель остановился на ширине %.3f м",
          result->position);
        return;
      }
    case Transition::kStallAborted:
      {
        auto result = makeResult(
          evaluation.result_reached, evaluation.result_stalled);
        active_goal_->abort(result);
        RCLCPP_ERROR(
          get_logger(),
          "Хвататель не достиг цели: width=%.3f, start=%.3f, "
          "closing_progress=%s, "
          "finger_motion_supports_contact=%s, left=%.3f, right=%.3f",
          result->position,
          evaluation.start_width.value_or(
            std::numeric_limits<double>::quiet_NaN()),
          evaluation.closing_progress ? "true" : "false",
          evaluation.finger_motion_supports_contact ? "true" : "false",
          last_state_->left_joint, last_state_->right_joint);
        active_goal_.reset();
        goal_reservation_.release();
        return;
      }
    case Transition::kGoalTimeout:
      abortActiveGoal("Истек timeout выполнения команды");
      return;
  }
}

void PandaGripperDriver::publishFeedback(
  const GripperGoalEvaluation & evaluation)
{
  if (!last_state_.has_value()) {
    return;
  }
  auto feedback = std::make_shared<GripperAction::Feedback>();
  feedback->position = last_state_->width();
  feedback->effort = last_state_->effort.value_or(0.0);
  feedback->reached_goal = evaluation.feedback_reached;
  feedback->stalled = evaluation.feedback_stalled;
  active_goal_->publish_feedback(feedback);
}

void PandaGripperDriver::publishWidthCommand(double width)
{
  command_pub_->publish(makePandaGripperWidthCommand(
      width, command_joint_count_,
      command_joint_min_position_, command_joint_max_position_));
}

void PandaGripperDriver::publishHoldPosition()
{
  if (!last_state_.has_value() ||
    SteadyClock::now() - last_state_->received_at > state_timeout_)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Нельзя отправить hold: нет свежего finger joint state");
    return;
  }

  command_pub_->publish(makePandaGripperHoldCommand(
      last_state_->left_joint, last_state_->right_joint,
      command_joint_count_,
      command_joint_min_position_, command_joint_max_position_));
}

}  // namespace franka_motion::detail
